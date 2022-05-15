package frc.robot.turret;

public class TurretConstants {
    public static final int TURRET_MOTOR_ID = 9;
    public static final int CALIBRATION_LIMIT_SWITCH_ID = 9;

    public static final double TOTAL_GEAR_RATIO = 1.0 / (5.23 * 5.23 * 1.0);
    public static final double TOTAL_SPROCKET_TOOTH_RATIO = 20.0 / 156.0;
    public static final double LIMIT_SWITCH_POSITION = 186.5; // In degrees

    public static final double SETPOINT_TOLERANCE = 1.0;
    public static final double LIMELIGHT_TOLERANCE = 0.2; // In degrees
    public static final double MOD_SPEED = 1.0;
    public static final double MOD_SPEED_MAX = 0.8;
    public static final double MANUAL_AIMING_MOD_SPEED = 4.0;
    public static final double CALIBRATION_MOD_SPEED = 0.17;

    public static final double LOW_LIMIT_DEGREES = -195.0;
    public static final double HIGH_LIMIT_DEGREES = 220.0;
    // The below is in absolute degrees relative to the turret
    // To be within frame perimeter, the turret must face backwards
    public static final double STOWED_DEGREES = 0.0;
    public static final double OFFSET_DEGREES = -4.0;

    public static final boolean BRAKE_WHEN_IDLE = true;
    public static final int CURRENT_LIMIT = 18;
}
