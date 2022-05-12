package frc.robot.Shooter;

public class TurretConstants {
    public static final double TOTAL_GEAR_RATIO = 1.0 / (5.23 * 5.23 * 1.0);
    public static final double TOTAL_SPROCKET_TOOTH_RATIO = 20.0 / 156.0;
    public static final double LIMIT_SWITCH_POSITION = 186.5; // In degrees

    public static final double SETPOINT_TOLERANCE = 1.0;

    public static final double MOD_SPEED = 1.0;
    public static final double MOD_SPEED_MAX = 0.8;

    public static final double LOW_LIMIT_DEGREES = -195.0;
    public static final double HIGH_LIMIT_DEGREES = 220.0;
    public static final double OFFSET_DEGREES = -4.0;

    public static final boolean BRAKE_WHEN_IDLE = true;
    public static final int CURRENT_LIMIT = 18;
}
