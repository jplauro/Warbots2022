package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controls.ControlConstants;
import frc.robot.util.sim.NavxWrapper;
import frc.robot.util.sim.RevEncoderSimWrapper;

public class Drivetrain extends SubsystemBase {
    private SimableCANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private SimableCANSparkMax[] motors;
    private DifferentialDrive drive;
    private AHRS navx;
    private RelativeEncoder leftEncoder, rightEncoder;
    private DifferentialDriveOdometry odometry;

    public Drivetrain() {
        this.initMotors();
        this.initDrive();
        this.initSensors();
    }

    public void curvatureInput(double speed, double rotation, boolean turnInPlace) {
        this.drive.curvatureDrive(speed, -rotation, turnInPlace);
    }

    public void setEncoderPos(double pos) {
        for (SimableCANSparkMax motor : this.motors) {
            motor.getEncoder().setPosition(pos);
        }
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public double getHeading() { // TODO: Remove Use Odom class
        return this.navx.getAngle();
    }

    public void tankDriveSet(double leftSpeed, double rightSpeed) {
        this.frontLeftMotor.set(leftSpeed);
        this.frontRightMotor.set(rightSpeed);
    }

    public void arcadeDriveSet(double speed, double rotation) {
        this.drive.arcadeDrive(speed, rotation);
    }

    // average the two encoders for this later
    public double getLeftSpeed() { // TODO: Remove Use Odom class
        return this.leftEncoder.getVelocity();
    }

    public double getLeftPosition() {// TODO: Remove Use Odom class
        return this.leftEncoder.getPosition();
    }

    public double getRightSpeed() {//TODO: Remove Use Odom class
        return this.rightEncoder.getVelocity();
    }

    public double getRightPosition() {//TODO: Remove Use Odom class
        return this.rightEncoder.getPosition();
    }

    public void setBrake(boolean brake) {
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        for (SimableCANSparkMax motor : this.motors) {
            motor.setIdleMode(mode);
        }
    }

    @Override
    public void periodic() {
        this.odometry.update(this.navx.getRotation2d(), this.leftEncoder.getPosition(), this.rightEncoder.getPosition());
        SmartDashboard.putNumber("Heading", this.navx.getYaw());
    }

    private void initMotors() {
        this.frontLeftMotor = new SimableCANSparkMax(DriveConstants.FRONT_LEFT_MOTOR_ID, MotorType.kBrushless);
        this.frontRightMotor = new SimableCANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
        this.rearLeftMotor = new SimableCANSparkMax(DriveConstants.REAR_LEFT_MOTOR_ID, MotorType.kBrushless);
        this.rearRightMotor = new SimableCANSparkMax(DriveConstants.REAR_RIGHT_MOTOR_ID, MotorType.kBrushless);

        this.motors = new SimableCANSparkMax[] {
            this.frontLeftMotor, this.frontRightMotor, this.rearLeftMotor, this.rearRightMotor
        };

        this.rearRightMotor.follow(this.frontRightMotor);
        this.rearLeftMotor.follow(this.frontLeftMotor);

        this.frontRightMotor.setInverted(false);
        this.frontLeftMotor.setInverted(true);

        for (SimableCANSparkMax motor : this.motors) {
            motor.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
        }

        this.frontRightMotor.setOpenLoopRampRate(ControlConstants.RAMP_RATE);
        this.frontLeftMotor.setOpenLoopRampRate(ControlConstants.RAMP_RATE);

        this.setBrake(DriveConstants.BRAKE_WHEN_IDLE);
    }

    private void initDrive() {
        this.drive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);
        this.drive.setMaxOutput(ControlConstants.SPEED_MAX);
    }

    private void initSensors() {
        this.navx = new AHRS(Port.kMXP);
        this.leftEncoder = this.frontLeftMotor.getEncoder();
        this.rightEncoder = this.frontRightMotor.getEncoder();
        this.odometry = new DifferentialDriveOdometry(this.navx.getRotation2d());
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.drivetrainSim.getCurrentDrawAmps();
        }
        return this.frontLeftMotor.getOutputCurrent() + this.frontRightMotor.getOutputCurrent()
            + this.rearLeftMotor.getOutputCurrent() + this.rearRightMotor.getOutputCurrent();
    }

    /**
     * Simulation Code
     */
    private NavxWrapper simGyro;
    private DifferentialDrivetrainSim drivetrainSim;
    private RevEncoderSimWrapper leftEncSim;
    private RevEncoderSimWrapper rightEncSim;
    private boolean simInit;

    private void initSim() {
        LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
            Constants.kSimDrivekVLinear, Constants.ksimDrivekALinear, 
            Constants.ksimDrivekVAngular, Constants.kSimDrivekAAngular);
        this.drivetrainSim = new DifferentialDrivetrainSim(
            drivetrainSystem, DCMotor.getNEO(2), Constants.gearRatio, Constants.kTrackwidthMeters,
            Units.inchesToMeters(Constants.wheelDiameterInInches / 2), null);

        // Setup Leader Motors
        this.leftEncSim = RevEncoderSimWrapper.create(this.frontLeftMotor);
        this.rightEncSim = RevEncoderSimWrapper.create(this.frontRightMotor);

        // Sim Motors
        this.simGyro = new NavxWrapper();
    }

    @Override
    public void simulationPeriodic() {
        if (!this.simInit) {
            this.initSim();
            this.simInit = true;
        }

        this.drivetrainSim.setInputs(this.frontLeftMotor.get() * RobotController.getInputVoltage(),
            this.frontRightMotor.get() * RobotController.getInputVoltage());
            this.drivetrainSim.update(Constants.kSimUpdateTime);

        this.leftEncSim.setDistance(this.drivetrainSim.getLeftPositionMeters());
        this.leftEncSim.setVelocity(this.drivetrainSim.getLeftVelocityMetersPerSecond());

        this.rightEncSim.setDistance(this.drivetrainSim.getRightPositionMeters());
        this.rightEncSim.setVelocity(this.drivetrainSim.getRightVelocityMetersPerSecond());

        this.simGyro.getYawGyro().setAngle(-this.drivetrainSim.getHeading().getDegrees()); // TODO add Gyo Vel support
    }
}
