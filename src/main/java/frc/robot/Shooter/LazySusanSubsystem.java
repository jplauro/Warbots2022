package frc.robot.Shooter;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class LazySusanSubsystem extends SubsystemBase {
    private final SimableCANSparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput calSwitch;
    private final Supplier<Pose2d> robotBasePose;
    
    private PIDController lazySusanPID;
    private double desiredRotation;
    private boolean isHubTracking;
    private boolean isGyroLocking;
    private double modSpeed = TurretConstants.MOD_SPEED;
    private double offsetDegrees = TurretConstants.OFFSET_DEGREES;
    private boolean isCal;

    private final double countsToDegreesFactor = TurretConstants.TOTAL_GEAR_RATIO * TurretConstants.TOTAL_SPROCKET_TOOTH_RATIO * 360.0;
    private final double kP = 0.038000, kI = 0.2, kD = 0.000400;
    private final double maxIntegrator = 0.7;

    public LazySusanSubsystem(Supplier<Pose2d> robotBasePose) {
        this.isGyroLocking = false;
        this.isHubTracking = false;
        this.robotBasePose = robotBasePose;
        this.motor = new SimableCANSparkMax(Constants.lazySusanID, MotorType.kBrushless);
        this.calSwitch = new DigitalInput(Constants.calSwitchID);
        this.motor.restoreFactoryDefaults();
        this.encoder = this.motor.getEncoder();
        encoder.setPositionConversionFactor(countsToDegreesFactor);
        this.encoder.setPosition(Constants.stowedDegrees);
        this.motor.setIdleMode(TurretConstants.BRAKE_WHEN_IDLE ? IdleMode.kBrake : IdleMode.kCoast);
        this.motor.setSmartCurrentLimit(TurretConstants.CURRENT_LIMIT);

        this.lazySusanPID = new PIDController(this.kP, this.kI, this.kD);
        this.lazySusanPID.setTolerance(TurretConstants.SETPOINT_TOLERANCE);
        this.lazySusanPID.setIntegratorRange(-this.maxIntegrator, this.maxIntegrator);
        this.isCal = false;

        SmartDashboard.putData("Turret/LazySusanPID", this.lazySusanPID);
        SmartDashboard.putNumber("Turret/Offset Degrees", this.offsetDegrees);
    }

    @Override
    public void periodic() {
        double degrees = this.isGyroLocking ? Rotation2d.fromDegrees(this.desiredRotation)
        .minus(this.robotBasePose.get().getRotation()).getDegrees() : this.desiredRotation;

        degrees = findClosestSolution(degrees, this.getRotationDegrees());
        this.lazySusanPID.setSetpoint(MathUtil.clamp(degrees, TurretConstants.LOW_LIMIT_DEGREES, TurretConstants.HIGH_LIMIT_DEGREES));
        double pidOutput = MathUtil.clamp(this.lazySusanPID.calculate(this.encoder.getPosition()), -1, 1);

        // Hard reverse turn if past safe boundaries
        if (this.encoder.getPosition() < TurretConstants.LOW_LIMIT_DEGREES) {
            pidOutput = MathUtil.clamp(pidOutput, 0, 1);
        }

        if (this.encoder.getPosition() > TurretConstants.HIGH_LIMIT_DEGREES) {
            pidOutput = MathUtil.clamp(pidOutput, -1, 0);
        }

        motor.set(MathUtil.clamp(
            pidOutput, -this.modSpeed * TurretConstants.MOD_SPEED_MAX, 
            this.modSpeed * TurretConstants.MOD_SPEED_MAX
        ));

        SmartDashboard.putNumber("Turret/Raw Encoder", encoder.getPosition());
        SmartDashboard.putNumber("Turret/Motor Percentage", motor.get());
        SmartDashboard.putBoolean("Turret/Is Calibrated", isCal);
        SmartDashboard.putBoolean("Turret/Calibration Switch", islimitSwitchPressed());
        SmartDashboard.putNumber("Turret/Desired Rotation", desiredRotation);
        SmartDashboard.putNumber("Turret/Setpoint", this.lazySusanPID.getSetpoint());
        SmartDashboard.putBoolean("Turret/Is Gyro Locking", isGyroLocking);
        SmartDashboard.putBoolean("Turret/Is Hub Tracking", isHubTracking);
        SmartDashboard.putNumber("Turret/ModSpeed", this.modSpeed); 
        SmartDashboard.putNumber("Turret/maxIntegrator", this.maxIntegrator);     
    }

    public double findClosestSolution(double targetRotation, double currentRotation){
        double solution2 = targetRotation - Math.signum(targetRotation) * 360;
        double distance1 = Math.abs(desiredRotation - this.getRotationDegrees());
        double distance2 = Math.abs(solution2 - this.getRotationDegrees());
        return distance2 < distance1 && solution2 > TurretConstants.LOW_LIMIT_DEGREES 
        && solution2 < TurretConstants.HIGH_LIMIT_DEGREES ? solution2 : targetRotation;
    }

    public double getOffsetDegrees() {
        return this.offsetDegrees;
    }
    
    public void setOffsetDegrees(double offsetDegrees) {
        this.offsetDegrees = offsetDegrees;
    }

    public void setSmartCurrentLimit(int currentLimit) {
        this.motor.setSmartCurrentLimit(currentLimit);
    }

    public double getEncoderPosition() {
        return this.encoder.getPosition();
    }

    public void setMotorMode(IdleMode mode) {
        this.motor.setIdleMode(mode);
    }

    public double getModSpeed() {
        return this.modSpeed;
    }

    public void setModSpeed(double modSpeed) {
        this.modSpeed = modSpeed;
    }

    public boolean getIsGyroLocking() {
        return this.isGyroLocking;
    }

    public void setIsGyroLocking(boolean isGyroLocking) {
        this.isGyroLocking = isGyroLocking;
        // Add an inverse offset to account for the degree position calculation method
        if (this.isGyroLocking) {
            this.setTurretPosition(this.robotBasePose.get().getRotation().plus(getDesiredRotation()));
        } else {
            this.setTurretPosition(getDesiredRotation().minus(this.robotBasePose.get().getRotation()));
        }
    }

    public boolean getIsHubTracking() {
        return this.isHubTracking;
    }

    public void setIsHubTracking(boolean isHubTracking) {
        this.isHubTracking = isHubTracking;
    }

    public boolean getIsCal() {
        return isCal;
    }

    public void setIsCal(boolean isCal) {
        this.isCal = isCal;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(this.encoder.getPosition());
    }

    public double getRotationDegrees() {
        return this.encoder.getPosition();
    }

    public Rotation2d getDesiredRotation() {
        return Rotation2d.fromDegrees(this.desiredRotation);
    }

    public double getDesiredRotationDegrees() {
        return this.desiredRotation;
    }

    public void setTurretPosition(Rotation2d rotation) {
        this.desiredRotation = rotation.getDegrees();
    }

    public void setTurretPositionDegrees(double rotation) {
        this.desiredRotation = MathUtil.clamp(
            rotation, TurretConstants.LOW_LIMIT_DEGREES, TurretConstants.HIGH_LIMIT_DEGREES
        );
    }

    public double getRawSetpoint() {
        return this.lazySusanPID.getSetpoint();
    }

    public boolean atTurretPosition() {
        return this.lazySusanPID.atSetpoint();
    }

    public void setEncoderPosition(double position) {
        this.encoder.setPosition(position);
    }

    public void setMotorSpeed(double speed) {
        this.motor.set(speed);
    }

    public void stop() {
        this.lazySusanPID.reset();
        this.lazySusanPID.setSetpoint(this.encoder.getPosition());
        this.setTurretPositionDegrees(this.encoder.getPosition());
    }

    public void setHomePosition() {
        this.setIsGyroLocking(false);
        this.setEncoderPosition(TurretConstants.LIMIT_SWITCH_POSITION);
        this.lazySusanPID.reset();
        this.lazySusanPID.setSetpoint(TurretConstants.LIMIT_SWITCH_POSITION);
        this.setTurretPositionDegrees(TurretConstants.LIMIT_SWITCH_POSITION);
        this.setIsCal(true);
    }

    public boolean islimitSwitchPressed() {
        return this.calSwitch.get();
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simlazySusan.getCurrentDrawAmps();
        }
        return this.motor.getOutputCurrent();
    }

    /**
     * Simulation Code
     */
    private DCMotorSim simlazySusan;
    private RevEncoderSimWrapper simEncoder;
    private boolean simInit = false;
    private double simlastpos = 0;
    private double simoffset = 0;

    private void initSim() {
        this.simlazySusan = new DCMotorSim(
            DCMotor.getNeo550(1), TurretConstants.TOTAL_GEAR_RATIO 
            * TurretConstants.TOTAL_SPROCKET_TOOTH_RATIO, Constants.kSimTurntableInertia
        );
        this.simEncoder = RevEncoderSimWrapper.create(this.motor);
    }

    @Override
    public void simulationPeriodic() {
        if (!this.simInit) {
            this.initSim();
            this.simInit = true;
        }
        this.simlazySusan.setInputVoltage(this.motor.get() * RobotController.getInputVoltage());
        this.simlazySusan.update(Constants.kSimUpdateTime);
        this.simEncoder.setVelocity(this.simlazySusan.getAngularVelocityRPM());
        if (this.simEncoder.getPosition() != this.simlastpos) {
            System.out.println(this.simEncoder.getPosition() + " " + this.simlastpos);
            simoffset = (this.simlazySusan.getAngularPositionRotations() * 360) - this.simEncoder.getPosition();
            System.out.println("Encoder Position changed" + this.simoffset);
        }

        this.simEncoder.setDistance((this.simlazySusan.getAngularPositionRotations() * 360) - this.simoffset);
        // TODO: Remove magic number 5 that represents the first gear reduction
        this.simlastpos = this.simEncoder.getPosition();
        SmartDashboard.putNumber("Turret/Velocity", this.encoder.getVelocity());
        SmartDashboard.putNumber("Turret/Sim rotation", this.simlazySusan.getAngularPositionRotations() * 360);
        SmartDashboard.putNumber("Turret/Setpoint", this.lazySusanPID.getSetpoint());
    }
}
