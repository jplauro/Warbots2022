package frc.robot.Climber;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class ClimberSubsystem extends SubsystemBase {
    private final SimableCANSparkMax leftMotor, rightMotor;
    private final SimableCANSparkMax[] motors;
    private final Solenoid armsSolenoid;
    private final Solenoid hooksSolenoid;
    private final RelativeEncoder encoder;
    private final SparkMaxLimitSwitch limitSwitch;
    private final DigitalInput proximitySensor;

    public ClimberSubsystem() {
        this.leftMotor = new SimableCANSparkMax(ClimberConstants.LEFT_WINCH_MOTOR_ID, MotorType.kBrushless);
        this.rightMotor = new SimableCANSparkMax(ClimberConstants.RIGHT_WINCH_MOTOR_ID, MotorType.kBrushless);
        this.armsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.ARMS_SOLENOID_ID);
        this.hooksSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.HOOKS_SOLENOID_ID);
        this.encoder = this.leftMotor.getEncoder();
        this.proximitySensor = new DigitalInput(ClimberConstants.PROXIMITY_SENSOR_ID);
        this.limitSwitch = this.leftMotor.getReverseLimitSwitch(Type.kNormallyOpen);

        this.motors = new SimableCANSparkMax[] {
            this.leftMotor, this.rightMotor
        };

        for (SimableCANSparkMax motor : this.motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(ClimberConstants.BRAKE_WHEN_IDLE ? IdleMode.kBrake : IdleMode.kCoast);
        }

        this.rightMotor.follow(this.leftMotor, true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber/Limit Switch", this.getWinchLimitSwitch());
    }

    public double getWinchSpeed() {
        return this.leftMotor.get();
    }

    public void setWinchSpeed(double speed) {
        this.leftMotor.set(speed);
    }

    public double getWinchPosition() {
        return this.encoder.getPosition();
    }

    public void setWinchPosition(double position) {
        this.encoder.setPosition(position);
    }

    public boolean getArmsSolenoid() {
        return this.armsSolenoid.get();
    }

    public boolean getHooksSolenoid() {
        return this.hooksSolenoid.get();
    }

    public void setArmsSolenoid(boolean state) {
        this.armsSolenoid.set(state);
    }

    public void setHangingSolenoid(boolean state) {
        this.hooksSolenoid.set(state);
    }

    public boolean getProximitySensor() {
        return this.proximitySensor.get();
    }

    public boolean getWinchLimitSwitch() {
        return this.limitSwitch.isPressed();
    }

    /**
     * Simulation Code
     */
    private boolean simInit;
    private ElevatorSim simWinch;
    private RevEncoderSimWrapper simEncoder;

    private void initSim() {
        this.simWinch = new ElevatorSim(DCMotor.getNEO(2), 21, Units.lbsToKilograms(3), 
        Units.inchesToMeters(0.5), 0, Units.feetToMeters(5.5));
        this.simEncoder = RevEncoderSimWrapper.create(this.rightMotor);
    }

    @Override
    public void simulationPeriodic() {
        if (!this.simInit) {
            this.initSim();
            this.simInit = true;
        }
        double rotations = this.simWinch.getPositionMeters() / Units.feetToMeters(5.5)
        * (ClimberConstants.WINCH_LIMIT_MAX + 1);
        
        this.simEncoder.setDistance(rotations); // TODO convert to rotations :/
        SmartDashboard.putNumber("Climber/Position", this.encoder.getPosition());
        SmartDashboard.putNumber("Climber/Motor Set", this.rightMotor.get());
        SmartDashboard.putNumber("Climber/Test Pos", this.simWinch.getPositionMeters());
    }
}
