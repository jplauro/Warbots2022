package frc.robot.shooter;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controls.ControlBoard;
import frc.robot.util.Limelight;
import frc.robot.util.RobotMath;
import frc.robot.util.sim.LimeLightPoseSim;
import frc.robot.util.sim.LimeLightSim;
import frc.robot.util.sim.RevEncoderSimWrapper;

public class ShooterSubsystem extends SubsystemBase {
    private final SimableCANSparkMax leftShooterMotor, rightShooterMotor;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final SimpleMotorFeedforward feedforward;
    private final PIDController leftPidController, rightPidController;

    private boolean isBackward;
    private boolean powerDecel = true;
    private double offsetSpeed = 0;

    private final double kS = -0.07488, kV = 0.12385, kA = 0.020886; //TODO: SysID characterize 
	private final double kP = 0.000001, kI = 0.003500, kD = 0.010000; // TODO: Tune PID loops more for lower RPMs

    // When setting any non-speed value, make sure to set the isBackward boolean!

    public ShooterSubsystem() {
        this.leftShooterMotor = new SimableCANSparkMax(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        this.rightShooterMotor = new SimableCANSparkMax(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        this.leftEncoder = this.leftShooterMotor.getEncoder();
        this.rightEncoder = this.rightShooterMotor.getEncoder();
        this.feedforward = new SimpleMotorFeedforward(this.kS, this.kV, this.kA);
        this.leftPidController = new PIDController(this.kP, this.kI, this.kD);
        this.rightPidController = new PIDController(this.kP, this.kI, this.kD);

        for (SimableCANSparkMax motor : new SimableCANSparkMax[] {this.leftShooterMotor, this.rightShooterMotor}) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(ShooterConstants.BRAKE_WHEN_IDLE ? IdleMode.kBrake : IdleMode.kCoast);
            motor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        }

        this.leftShooterMotor.setInverted(true);
        this.leftPidController.setTolerance(ShooterConstants.POSITION_TOLERANCE, ShooterConstants.VELOCITY_TOLERANCE);
        this.rightPidController.setTolerance(ShooterConstants.POSITION_TOLERANCE, ShooterConstants.VELOCITY_TOLERANCE);
  
        SmartDashboard.putData("Shooter/Left PID Controller", this.leftPidController);
        SmartDashboard.putData("Shooter/Right PID Controller", this.rightPidController);
    }

    @Override
    public void periodic() {
        boolean hasTargetAndInRange = Limelight.hasTarget() && ShooterConstants.RPM_MAP.isKeyInBounds(Limelight.getTY());

        ControlBoard.setDriverHighFreqRumble(hasTargetAndInRange);
        ControlBoard.setOperatorHighFreqRumble(hasTargetAndInRange);

        this.leftPidController.calculate(this.leftEncoder.getVelocity());
        this.rightPidController.calculate(this.rightEncoder.getVelocity());

        double leftOutputVoltage = this.leftPidController.calculate(this.leftEncoder.getVelocity()) 
        + this.feedforward.calculate(this.leftPidController.getSetpoint() / 60);

        double rightOutputVoltage = this.rightPidController.calculate(this.rightEncoder.getVelocity())
        + this.feedforward.calculate(this.rightPidController.getSetpoint() / 60);

        if (this.isBackward) {
            this.leftShooterMotor.setVoltage(-13);
            this.rightShooterMotor.setVoltage(-13);
        } else {
            this.leftShooterMotor.setVoltage(MathUtil.clamp(leftOutputVoltage, 
                this.powerDecel || this.leftPidController.getSetpoint() <= 0 ? 0 : -13, 13));
            this.rightShooterMotor.setVoltage(MathUtil.clamp(rightOutputVoltage, 
                this.powerDecel || this.rightPidController.getSetpoint() <= 0 ? 0 : -13, 13));
        }

        ControlBoard.setDriverLowFreqRumble(hasTargetAndInRange && this.isWithinTolerance());
        ControlBoard.setOperatorLowFreqRumble(hasTargetAndInRange && this.isWithinTolerance());

        SmartDashboard.putBoolean("Shooter/Has Target", hasTargetAndInRange);
        SmartDashboard.putBoolean("Shooter/Ready", hasTargetAndInRange && this.isWithinTolerance());
        SmartDashboard.putNumber("Shooter/Left RPM", this.leftEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Left At Target", this.leftPidController.atSetpoint() ? 5000 : 0);
        SmartDashboard.putNumber("Shooter/Right RPM", this.rightEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Right At Target", this.rightPidController.atSetpoint() ? 5000 : 0);
        SmartDashboard.putNumber("Shooter/Left Setpoint", this.leftPidController.getSetpoint());
        SmartDashboard.putNumber("Shooter/Right Setpoint", this.rightPidController.getSetpoint());
    }

    public boolean isWithinTolerance() {
        return RobotMath.isWithinTolerance(
            this.getRPM(), 
            this.getTargetRPM(), 
            ShooterConstants.VIBRATION_TOLERANCE
        );
    }

    public double getSpeed() {
        return (this.leftShooterMotor.get() + this.rightShooterMotor.get()) / 2;
    }

    public void setSpeed(double speed) {
        this.isBackward = speed < 0;
        this.leftShooterMotor.set(speed);
        this.rightShooterMotor.set(speed);
    }

    public double getOffsetSpeed() {
        return this.offsetSpeed;
    }

    public void setOffsetSpeed(double offsetSpeed) {
        this.offsetSpeed = offsetSpeed;
    }
    
    public boolean atTargetRPM() {
        return this.leftPidController.atSetpoint() && this.rightPidController.atSetpoint();
    }

    public double getLeftRPM() {
        return this.leftEncoder.getVelocity();
    }

    public double getRightRPM() {
        return this.rightEncoder.getVelocity();
    }

    public double getRPM() {
        return (this.getLeftRPM() + this.getRightRPM()) / 2;
    }

    public void stopMotors() {
		this.setTargetRPM(0);
    }

    public void setTargetRPM(double rpm) {
        this.isBackward = false;
        this.leftPidController.setSetpoint(rpm);
        this.rightPidController.setSetpoint(rpm);
    }

    public double getTargetRPM() {
        return this.leftPidController.getSetpoint();
    }

    public void setIsBackward(boolean isBackward) {
        this.isBackward = isBackward;
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simFlywheelLeft.getCurrentDrawAmps() + this.simFlywheelRight.getCurrentDrawAmps();
        } else {
            return this.rightShooterMotor.getOutputCurrent() + this.leftShooterMotor.getOutputCurrent();
        }
    }

    /**
     * Simulation Code
     */
    private FlywheelSim simFlywheelLeft, simFlywheelRight;
    private RevEncoderSimWrapper leftEncSim, rightEncSim;
    private boolean simInit;
    private LimeLightSim simLimeLight;
    public LimeLightPoseSim posSim;

    private void initSim() {
        this.simFlywheelLeft = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.SHOOTER_GEAR_RATIO, Constants.kSimShooterInertia);
        this.simFlywheelRight = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.SHOOTER_GEAR_RATIO, Constants.kSimShooterInertia);
        this.leftEncSim = RevEncoderSimWrapper.create(this.leftShooterMotor);
        this.rightEncSim = RevEncoderSimWrapper.create(this.rightShooterMotor);
        this.simLimeLight = new LimeLightSim();
        Pose2d hubPos = new Pose2d(7.940, 4.08, new Rotation2d());
        this.posSim = new LimeLightPoseSim(this.simLimeLight, hubPos, 
            Constants.LIMELIGHT_HEIGHT, Constants.HUB_HEIGHT, Constants.LIMELIGHT_ANGLE);
    }

    @Override
    public void simulationPeriodic() {
        if (!this.simInit) {
            this.initSim();
            this.simInit = true;
        }

        this.simFlywheelLeft.setInputVoltage(this.leftShooterMotor.get() * RobotController.getInputVoltage());
        this.simFlywheelRight.setInputVoltage(this.rightShooterMotor.get() * RobotController.getInputVoltage());
        this.simFlywheelLeft.update(Constants.kSimUpdateTime);
        this.simFlywheelRight.update(Constants.kSimUpdateTime);
        this.leftEncSim.setVelocity(this.simFlywheelLeft.getAngularVelocityRPM());
        this.rightEncSim.setVelocity(this.simFlywheelRight.getAngularVelocityRPM());
    }
}
