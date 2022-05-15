package frc.robot.Turret;

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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class TurretSubsystem extends SubsystemBase {
    private final SimableCANSparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput limitSwitch;
    private final PIDController pidController;
    private final Supplier<Pose2d> robotBasePose;

    private double desiredRotation;
    private double modSpeed = TurretConstants.MOD_SPEED;
    private double offsetDegrees = TurretConstants.OFFSET_DEGREES;
    private boolean isHubTracking;
    private boolean isGyroLocking;
    private boolean isCalibrated;

    private final double countsToDegreesFactor = TurretConstants.TOTAL_GEAR_RATIO
    * TurretConstants.TOTAL_SPROCKET_TOOTH_RATIO * 360.0;
    private final double kP = 0.038000, kI = 0.2, kD = 0.000400;
    private final double maxIntegrator = 0.7;

    public TurretSubsystem(Supplier<Pose2d> robotBasePose) {
        this.motor = new SimableCANSparkMax(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.limitSwitch = new DigitalInput(TurretConstants.CALIBRATION_LIMIT_SWITCH_ID);
        this.pidController = new PIDController(this.kP, this.kI, this.kD);
        this.robotBasePose = robotBasePose;

        this.motor.restoreFactoryDefaults();
        this.motor.setIdleMode(TurretConstants.BRAKE_WHEN_IDLE ? IdleMode.kBrake : IdleMode.kCoast);
        this.motor.setSmartCurrentLimit(TurretConstants.CURRENT_LIMIT);

        this.encoder.setPositionConversionFactor(this.countsToDegreesFactor); // Encoder counts are degrees
        this.encoder.setPosition(TurretConstants.STOWED_DEGREES);

        this.pidController.setTolerance(TurretConstants.SETPOINT_TOLERANCE);
        this.pidController.setIntegratorRange(-this.maxIntegrator, this.maxIntegrator);

        SmartDashboard.putData("Turret/PID Controller", this.pidController);
    }

    @Override
    public void periodic() {
        if (this.getIsCalibrated()) {
            double degrees = this.findClosestSolution(this.isGyroLocking ? Rotation2d.fromDegrees(this.desiredRotation)
            .minus(this.robotBasePose.get().getRotation()).getDegrees() : this.desiredRotation, this.getRotationDegrees());

            this.pidController.setSetpoint(MathUtil.clamp(degrees, 
                TurretConstants.LOW_LIMIT_DEGREES, TurretConstants.HIGH_LIMIT_DEGREES));

            double pidOutput = MathUtil.clamp(this.pidController.calculate(this.encoder.getPosition()), -1, 1);

            // These prevent moving into an unsafe position
            if (this.encoder.getPosition() < TurretConstants.LOW_LIMIT_DEGREES) {
                pidOutput = MathUtil.clamp(pidOutput, 0, 1);
            }

            if (this.encoder.getPosition() > TurretConstants.HIGH_LIMIT_DEGREES) {
                pidOutput = MathUtil.clamp(pidOutput, -1, 0);
            }

            this.motor.set(MathUtil.clamp(
                pidOutput, -this.modSpeed * TurretConstants.MOD_SPEED_MAX, 
                this.modSpeed * TurretConstants.MOD_SPEED_MAX
            ));
        }

        SmartDashboard.putNumber("Turret/Raw Encoder", this.encoder.getPosition());
        SmartDashboard.putNumber("Turret/Motor Percentage", this.motor.get());
        SmartDashboard.putBoolean("Turret/Is Calibrated", this.isCalibrated);
        SmartDashboard.putBoolean("Turret/Calibration Switch", this.getLimitSwitch());
        SmartDashboard.putNumber("Turret/Desired Rotation", this.desiredRotation);
        SmartDashboard.putNumber("Turret/Setpoint", this.pidController.getSetpoint());
        SmartDashboard.putBoolean("Turret/Is Gyro Locking", this.isGyroLocking);
        SmartDashboard.putBoolean("Turret/Is Hub Tracking", this.isHubTracking);
        SmartDashboard.putNumber("Turret/Mod Speed", this.modSpeed);
        SmartDashboard.putNumber("Turret/Max Integrator", this.maxIntegrator);
        SmartDashboard.putNumber("Turret/Offset Degrees", this.offsetDegrees);
    }

    private double findClosestSolution(double targetRotation, double currentRotation) {
        double distance1 = Math.abs(targetRotation - currentRotation);
        double solution2 = targetRotation - Math.signum(targetRotation) * 360.0;
        double distance2 = Math.abs(solution2 - currentRotation);

        return distance2 < distance1 && solution2 > TurretConstants.LOW_LIMIT_DEGREES 
        && solution2 < TurretConstants.HIGH_LIMIT_DEGREES ? solution2 : targetRotation;
    }

    public void setSmartCurrentLimit(int currentLimit) {
        this.motor.setSmartCurrentLimit(currentLimit);
    }

    public IdleMode getMotorMode() {
        return this.motor.getIdleMode();
    }

    public void setMotorMode(IdleMode mode) {
        this.motor.setIdleMode(mode);
    }

    public double getEncoderPosition() {
        return this.encoder.getPosition();
    }

    public boolean getLimitSwitch() {
        return this.limitSwitch.get();
    }

    public double getModSpeed() {
        return this.modSpeed;
    }

    public void setModSpeed(double modSpeed) {
        this.modSpeed = modSpeed;
    }

    public double getOffsetDegrees() {
        return this.offsetDegrees;
    }

    public void setOffsetDegrees(double offsetDegrees) {
        this.offsetDegrees = offsetDegrees;
    }

    public boolean getIsGyroLocking() {
        return this.isGyroLocking;
    }

    public void setIsGyroLocking(boolean isGyroLocking) {
        if (isGyroLocking != this.isGyroLocking) {
            this.isGyroLocking = isGyroLocking;

            if (this.isGyroLocking) {
                this.setTurretPosition(this.robotBasePose.get().getRotation().plus(this.getDesiredRotation()));
            } else {
                this.setTurretPosition(this.getDesiredRotation().minus(this.robotBasePose.get().getRotation()));
            }
        }
    }

    public boolean getIsHubTracking() {
        return this.isHubTracking;
    }

    public void setIsHubTracking(boolean isHubTracking) {
        this.isHubTracking = isHubTracking;
    }

    public boolean getIsCalibrated() {
        return this.isCalibrated;
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
        return this.pidController.getSetpoint();
    }

    public boolean atTurretPosition() {
        return this.pidController.atSetpoint();
    }

    public void stop() {
        this.pidController.reset();
        this.pidController.setSetpoint(this.encoder.getPosition());
        this.setTurretPositionDegrees(this.encoder.getPosition());
    }

    public void setHomePosition() {
        this.setIsGyroLocking(false);
        this.encoder.setPosition(TurretConstants.LIMIT_SWITCH_POSITION);
        this.pidController.reset();
        this.pidController.setSetpoint(TurretConstants.LIMIT_SWITCH_POSITION);
        this.setTurretPositionDegrees(TurretConstants.LIMIT_SWITCH_POSITION);
        this.isCalibrated = true;
    }

    /**
     * Simulation Code
     */
    private DCMotorSim simTurret;
    private RevEncoderSimWrapper simEncoder;
    private boolean simInit;
    private double simLastPos;
    private double simOffset;

    private void initSim() {
        this.simTurret = new DCMotorSim(DCMotor.getNeo550(1), 
            this.countsToDegreesFactor, Constants.kSimTurntableInertia);

        this.simEncoder = RevEncoderSimWrapper.create(this.motor);
    }

    @Override
    public void simulationPeriodic() {
        if (!this.simInit) {
            this.initSim();
            this.simInit = true;
        }

        this.simTurret.setInputVoltage(this.motor.get() * RobotController.getInputVoltage());
        this.simTurret.update(Constants.kSimUpdateTime);
        this.simEncoder.setVelocity(this.simTurret.getAngularVelocityRPM());

        if (this.simEncoder.getPosition() != this.simLastPos) {
            System.out.println(this.simEncoder.getPosition() + " " + this.simLastPos);
            this.simOffset = (this.simTurret.getAngularPositionRotations()) - this.simEncoder.getPosition();
            System.out.println("Turret Encoder Position Changed" + this.simOffset);
        }

        this.simEncoder.setDistance((this.simTurret.getAngularPositionRotations()) - this.simOffset);
        // TODO: Remove magic number 5 that represents the first gear reduction
        this.simLastPos = this.simEncoder.getPosition();

        SmartDashboard.putNumber("Turret/Velocity", this.encoder.getVelocity());
        SmartDashboard.putNumber("Turret/Sim Rotation", this.simTurret.getAngularPositionRotations());
        SmartDashboard.putNumber("Turret/Setpoint", this.pidController.getSetpoint());
    }
}
