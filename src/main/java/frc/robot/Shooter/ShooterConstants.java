package frc.robot.Shooter;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.Util.LinearInterpolationMap;

public class ShooterConstants {
    public static final int LEFT_SHOOTER_MOTOR_ID = 5;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 6;
    public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kMXP;

    public static final double SHOOTER_GEAR_RATIO = 1.0;

    public static final boolean BRAKE_WHEN_IDLE = false;
    public static final int CURRENT_LIMIT = 45;
    public static final double POSITION_TOLERANCE = 10.0;
    public static final double VELOCITY_TOLERANCE = 10.0;
    public static final int MIN_COLOR_SENSOR_PROXIMITY = 100;

    public static final double SHOOTER_RPM_MIN = 0.0;
    public static final double SHOOTER_RPM_MAX = 5500.0;
    public static final double LOW_POWERED_SHOT_RPM = 1200.0;
    public static final double MAX_SHOOTING_DISTANCE = 4.0;
    public static final double VIBRATION_TOLERANCE = 0.03;

    public static final LinearInterpolationMap RPM_MAP = new LinearInterpolationMap() {{
        put(7.7, 2458.0);
        put(-0.4, 2682.0);
        put(-6.7, 2990.0);
        put(-12.3, 3375.0);
        put(-16.38, 3663.0);
        put(-19.09, 3930.0);
        put(-20.39, 4302.0);
        put(-22.13, 4575.0);
    }};
}
