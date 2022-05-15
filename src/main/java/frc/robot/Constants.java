package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double LIMELIGHT_ANGLE = 40; // Degrees
    public static final double LIMELIGHT_HEIGHT = 0.69; // Meters
    public static final double HUB_HEIGHT = 2.7178; // Meters

    // Ramsete Variables
    public static final double ksVolts = 0.13271;
    public static final double kvVoltSecondsPerMeter = 2.1525;
    public static final double kaVoltSecondsSquaredPerMeter = 0.40778;
    public static final double kPDriveVel = 2.9285;
    public static final double kTrackwidthMeters = 0.64;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Simulation Variables
    public static final double wheelDiameterInInches = 6;
    public static final double gearRatio = 0.12;
    public static final double kSimDrivekVLinear = 1.98;
    public static final double ksimDrivekALinear = 0.2;
    public static final double ksimDrivekVAngular = 1.5;
    public static final double kSimDrivekAAngular = 0.3;
    public static final double kSimShooterInertia = 0.5 * Units.lbsToKilograms(1.5) * Math.pow(Units.inchesToMeters(4.0 / 2.0), 2.0);
    public static final double kSimTurntableInertia = 0.5 * Units.lbsToKilograms(0.1) * Math.pow(Units.inchesToMeters(3.0), 2.0); //TODO: switch to ring formula vs disk
    public static final double kSimTurntableGearRatio = 500.0;
    public static final double kSimUpdateTime = 0.02;
    public static final double kSimRobotWeight = Units.lbsToKilograms(120.0);
    public static final double kSimClimberGearRatio = 1.0;
    public static final double kSimClimberDrumSize = 0.2;
    public static final double ksimClimberMaxHeight = Units.feetToMeters(6.0);
}
