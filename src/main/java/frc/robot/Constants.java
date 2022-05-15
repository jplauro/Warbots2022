package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double ksVolts = 0.13271;// 0.16249 0.13271
    public static final double kvVoltSecondsPerMeter = 2.1525;// 3.2952 2.1525
    public static final double kaVoltSecondsSquaredPerMeter = 0.40778;// 0.30496 0.40778

    public static final double kPDriveVel = 2.9285;// 3.869//2.0146

    public static final double kTrackwidthMeters = 0.64;// 0.5207
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    // Reasonable baseline values for a RAMETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;// 0.5832 2
    public static final double kRamseteZeta = 0.7;// 0.7
    // It seems that the meter value is three times what it should be.

    public static final double wheelDiameterInInches = 6; // TODO: Be more clear on if this is A drive wheel or shooter wheel
    public static final double inchesToMetersFactor = 0.0254; // TODO: remove use Units.inchestoMeter
    public static final double gearRatio = 0.12;

    public static final double motorRotationsPerTurntableRotation = 104.0;

    public static final double limelightHeight = 0.69; // based on last year's robot
    public static final double hubHeight = 2.7178;
    public static final double limelightAngle = 40; // mount angle of the limelight
    public static final double metersToFeet = 3.28084;

    // Differential Constants (very cool)
    public static final double diffConstLS = 0.014;//0.012 0.014
    public static final double diffConstAutoLS = 0.040;
    public static final double diffConstShooter = 9 * Math.pow(10, -6);
    public static final double diffConstKeepPosition = 0.00001; // TODO: Test this constant and optimize it

    // Sim Vars
    public static final double kSimGearRatio = Constants.gearRatio;
    public static final double kSimTrackwidthMeters = Constants.kTrackwidthMeters;
    public static final double kSimDrivekVLinear = 1.98;
    public static final double ksimDrivekALinear = 0.2;
    public static final double ksimDrivekVAngular = 1.5;
    public static final double kSimDrivekAAngular = 0.3;
    public static final double kSimShooterInertia = 0.5*Units.lbsToKilograms(1.5)*Math.pow(Units.inchesToMeters(4/2),2); //1/2*M*R^2 
    public static final double kSimTurntableInertia = 0.5*Units.lbsToKilograms(0.1)*Math.pow(Units.inchesToMeters(3),2); //TODO: switch to ring formula vs disk
    public static final double kSimTurntableGearRatio = 500.0;//5*(156.0/16.0);
    public static final double kSimUpdateTime = 0.02;
    public static final double kSimRobotWeight = Units.lbsToKilograms(120);
    public static final double kSimClimberGearRatio = 1;
    public static final double kSimClimberDrumSize = 0.2;
    public static final double ksimClimberMaxHeight = Units.feetToMeters(6);
}
