package frc.robot.Drive;

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
import frc.robot.Util.sim.NavxWrapper;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class Drivetrain extends SubsystemBase {
    private SimableCANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private SimableCANSparkMax[] motors;
    private DifferentialDrive drive;
    private AHRS navx;
    private RelativeEncoder leftFrontEncoder, rightFrontEncoder;
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
        return this.leftFrontEncoder.getVelocity();
    }

    public double getLeftPosition() {// TODO: Remove Use Odom class
        return this.leftFrontEncoder.getPosition();
    }

    public double getRightSpeed() {//TODO: Remove Use Odom class
        return this.rightFrontEncoder.getVelocity();
    }

    public double getRightPosition() {//TODO: Remove Use Odom class
        return this.rightFrontEncoder.getPosition();
    }

    public void setBrake(boolean brake) {
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        for (SimableCANSparkMax motor : this.motors) {
            motor.setIdleMode(mode);
        }
    }

    @Override
    public void periodic() {
        this.odometry.update(this.navx.getRotation2d(), this.leftFrontEncoder.getPosition(), this.rightFrontEncoder.getPosition());
        SmartDashboard.putNumber("Heading", this.navx.getYaw());
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
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

        this.frontRightMotor.setOpenLoopRampRate(DriveConstants.RAMP_RATE);
        this.frontLeftMotor.setOpenLoopRampRate(DriveConstants.RAMP_RATE);

        this.setBrake(DriveConstants.BRAKE_WHEN_IDLE);
    }

    private void initDrive() {
        this.drive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);
        this.drive.setMaxOutput(DriveConstants.SPEED_MAX);
    }

    private void initSensors() {
        this.navx = new AHRS(Port.kMXP);
        this.leftFrontEncoder = this.frontLeftMotor.getEncoder();
        this.rightFrontEncoder = this.frontRightMotor.getEncoder();
        this.odometry = new DifferentialDriveOdometry(this.navx.getRotation2d());
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.m_drivetrainSimulator.getCurrentDrawAmps();
        }
        return this.frontLeftMotor.getOutputCurrent() + this.frontRightMotor.getOutputCurrent()
            + this.rearLeftMotor.getOutputCurrent() + this.rearRightMotor.getOutputCurrent();
    }

    /**
     * Simulation Code
     */
    private NavxWrapper simGyro;
    private DifferentialDrivetrainSim m_drivetrainSimulator;
    private RevEncoderSimWrapper leftencsim;
    private RevEncoderSimWrapper rightencsim;
    private boolean simInit = false;

    private void initSim() {
        LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
                Constants.kSimDrivekVLinear,
                Constants.ksimDrivekALinear, Constants.ksimDrivekVAngular,
                Constants.kSimDrivekAAngular);
        this.m_drivetrainSimulator = new DifferentialDrivetrainSim(
                m_drivetrainSystem, DCMotor.getNEO(2), Constants.gearRatio, Constants.kTrackwidthMeters,
                Units.inchesToMeters(Constants.wheelDiameterInInches / 2), null);

        // Setup Leader Motors
        this.leftencsim = RevEncoderSimWrapper.create(this.frontLeftMotor);
        this.rightencsim = RevEncoderSimWrapper.create(this.frontRightMotor);

        // Sim Motors
        this.simGyro = new NavxWrapper();
    }

    @Override
    public void simulationPeriodic() {
        if (!this.simInit) {
            initSim();
            this.simInit = true;
        }
        this.m_drivetrainSimulator.setInputs(
                this.frontLeftMotor.get() * RobotController.getInputVoltage(),
                this.frontRightMotor.get() * RobotController.getInputVoltage());
                this.m_drivetrainSimulator.update(Constants.kSimUpdateTime);
        this.leftencsim.setDistance(this.m_drivetrainSimulator.getLeftPositionMeters());
        this.leftencsim.setVelocity(this.m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

        this.rightencsim.setDistance(this.m_drivetrainSimulator.getRightPositionMeters());
        this.rightencsim.setVelocity(this.m_drivetrainSimulator.getRightVelocityMetersPerSecond());

        this.simGyro.getYawGyro().setAngle(-this.m_drivetrainSimulator.getHeading().getDegrees()); // TODO add Gyo Vel support
    }
}
