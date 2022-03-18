// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.NavxWrapper;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class Drivetrain extends SubsystemBase {
    public enum Motor {
        RIGHT_BACK, RIGHT_FRONT, LEFT_BACK, LEFT_FRONT;
    }

    private final EnumMap<Motor, CANSparkMax> motors = new EnumMap<>(Motor.class);
    private final EnumMap<Motor, RelativeEncoder> encoders = new EnumMap<>(Motor.class);
    private final DifferentialDrive diffDrive;

    // The gyro sensor
    private final AHRS gyro;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    public Drivetrain() {
        this.motors.putAll(Map.of(
            Motor.RIGHT_BACK, new SimableCANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless),
            Motor.RIGHT_FRONT, new SimableCANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless),
            Motor.LEFT_BACK, new SimableCANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless),
            Motor.LEFT_FRONT, new SimableCANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless)
        ));

        double conversionFactor = Constants.gearRatio * Constants.wheelDiameterInInches * Constants.inchesToMetersFactor * Math.PI;

        for (Motor motor : Motor.values()) {
            CANSparkMax canSparkMax = this.motors.get(motor);
            RelativeEncoder encoder = canSparkMax.getEncoder();
            this.encoders.put(motor, encoder);
            canSparkMax.restoreFactoryDefaults();
            canSparkMax.setIdleMode(Constants.defaultIdleMode);
            canSparkMax.setOpenLoopRampRate(Constants.rampRate);
            canSparkMax.setSmartCurrentLimit(Constants.currentLimit);
            encoder.setPositionConversionFactor(conversionFactor);
            canSparkMax.burnFlash();
        }

        this.getMotor(Motor.RIGHT_BACK).follow(this.getMotor(Motor.RIGHT_FRONT));
        this.getMotor(Motor.LEFT_BACK).follow(this.getMotor(Motor.LEFT_FRONT));

        this.diffDrive = new DifferentialDrive(
            this.getMotor(Motor.RIGHT_FRONT), 
            this.getMotor(Motor.LEFT_FRONT)
        );

        this.diffDrive.setDeadband(Constants.deadband);
        this.getMotor(Motor.LEFT_FRONT).setInverted(true);

        this.gyro = new AHRS(SerialPort.Port.kMXP);
        this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());
    }

    private <T> void loopMotorArray(BiConsumer<Motor, T> function, Motor[] motors, T[] values) {
        // Specifying no motors returns all of them
        Motor[] motorArray = motors.length == 0 ? Motor.values() : motors;
        for (int i = 0; i < motorArray.length; i++) function.accept(motorArray[i], values[i]);
    }

    @SuppressWarnings("unchecked")
    private <T> void loopMotorArray(BiConsumer<Motor, T> function, Map<Motor, T> motors) {
        this.loopMotorArray(function, motors.keySet().toArray(Motor[]::new), (T[]) motors.values().toArray());
    }

    private void loopMotorArray(Consumer<Motor> function, Motor[] motors) {
        this.loopMotorArray((motor, object) -> function.accept(motor), motors, new Object[motors.length]);
    }

    private void loopMotorArray(Consumer<Motor> function, List<Motor> motors) {
        this.loopMotorArray(function, motors.toArray(Motor[]::new));
    }

    public CANSparkMax getMotor(Motor motor) {
        return this.motors.get(motor);
    }

    public RelativeEncoder getEncoder(Motor motor) {
        return this.encoders.get(motor);
    }

    public void setOpenLoopRampRate(double rampRate, Motor...motors) {
        this.loopMotorArray((motor) -> this.getMotor(motor).setOpenLoopRampRate(rampRate), motors);
    }

    public void setIdleMode(IdleMode mode, Motor...motors) {
        this.loopMotorArray((motor) -> this.getMotor(motor).setIdleMode(mode), motors);
    }

    public Pose2d getPose() {
        if (RobotBase.isSimulation()) {
            return this.m_drivetrainSimulator.getPose();
        } else {
            return this.odometry.getPoseMeters();
        }
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            this.getEncoder(Motor.LEFT_BACK).getVelocity(), 
            this.getEncoder(Motor.RIGHT_BACK).getVelocity()
        );
    }

    public double getHeading() {
        return this.gyro.getRotation2d().getDegrees();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        for (Motor motor : Motor.values()) {
            this.encoders.get(motor).setPosition(0);
        }
    }

    public void resetOdometry(Pose2d pose) {
        this.resetEncoders();
        this.odometry.resetPosition(pose, gyro.getRotation2d());
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.loopMotorArray((motor, volts) -> this.getMotor(motor).setVoltage(volts),
        Map.of(Motor.LEFT_FRONT, leftVolts, Motor.RIGHT_FRONT, rightVolts));
        this.diffDrive.feed();
    }

    public void tankDriveSet(double leftSpeed, double rightSpeed) {
        this.loopMotorArray((motor, speed) -> this.getMotor(motor).set(speed),
        Map.of(Motor.LEFT_FRONT, leftSpeed, Motor.RIGHT_FRONT, rightSpeed));
    }

    public void motorDrive(Motor motor, double speed) {
        this.getMotor(motor).set(speed);
    }

    public void setEncoderPos(Motor motor, double position) {
        this.getEncoder(motor).setPosition(position);
    }

    public double getEncoderPos(Motor motor) {
        return this.getEncoder(motor).getPosition();
    }

    public void curvatureInput(double speed, double rotation, boolean allowTurnInPlace) {
        this.diffDrive.curvatureDrive(speed, rotation, allowTurnInPlace);
    }

    public double getRPM(Motor motor) {
        return this.getEncoder(motor).getVelocity() * Constants.gearRatio;
    }

    @Override
    public void periodic() {
        this.odometry.update(
            this.gyro.getRotation2d(), 
            this.getEncoder(Motor.LEFT_BACK).getPosition(), 
            this.getEncoder(Motor.RIGHT_BACK).getPosition()
        );
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.m_drivetrainSimulator.getCurrentDrawAmps();
        } else {
            return this.getMotor(Motor.LEFT_FRONT).getOutputCurrent() +
            this.getMotor(Motor.LEFT_BACK).getOutputCurrent() +
            this.getMotor(Motor.RIGHT_FRONT).getOutputCurrent() + 
            this.getMotor(Motor.RIGHT_BACK).getOutputCurrent();
        }
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
        LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(Constants.kSimDrivekVLinear,
            Constants.ksimDrivekALinear, Constants.ksimDrivekVAngular,
            Constants.kSimDrivekAAngular);
        m_drivetrainSimulator = new DifferentialDrivetrainSim(
            m_drivetrainSystem, DCMotor.getNEO(2), Constants.gearRatio, Constants.kTrackwidthMeters,
            Units.inchesToMeters(Constants.wheelDiameterInInches / 2), null);

        // Setup Leader Motors
        this.leftencsim = RevEncoderSimWrapper.create(this.motors.get(Motor.LEFT_FRONT));
        this.rightencsim = RevEncoderSimWrapper.create(this.motors.get(Motor.RIGHT_FRONT));

        // Sim Motors
        simGyro = new NavxWrapper();
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        m_drivetrainSimulator.setInputs(
            this.motors.get(Motor.LEFT_FRONT).get() * RobotController.getInputVoltage(),
            this.motors.get(Motor.RIGHT_FRONT).get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(Constants.kSimUpdateTime);
        this.leftencsim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        this.leftencsim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        this.rightencsim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        this.rightencsim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        this.simGyro.getYawGyro().setAngle(-m_drivetrainSimulator.getHeading().getDegrees()); // TODO add Gyro Vel support
    }
}
