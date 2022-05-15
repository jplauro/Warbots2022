package frc.robot.Shooter;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FiringPins extends SubsystemBase {
    private final Solenoid firingPinsSolenoid;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;

    private final Color blueTarget = new Color(0.15, 0.4, 0.45);
    private final Color redTarget = new Color(0.5, 0.36, 0.14);

    public FiringPins() {
        this.firingPinsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        this.colorSensor = new ColorSensorV3(ShooterConstants.COLOR_SENSOR_PORT);
        this.colorMatcher = new ColorMatch();
        
        this.firingPinsSolenoid.set(false);
        this.colorMatcher.addColorMatch(this.blueTarget);
        this.colorMatcher.addColorMatch(this.redTarget);
    }

    public void extendFiringPinsSolenoid() {
        this.firingPinsSolenoid.set(true);
    }

    public void retractFiringPinsSolenoid() {
        this.firingPinsSolenoid.set(false);
    }

    public Color getDetectedColor() {
        return this.colorSensor.getColor();
    }

    public ColorMatchResult getColorMatch() {
        return this.colorMatcher.matchClosestColor(this.getDetectedColor());
    }

    public Color getMatchedColor() {
        return this.getColorMatch().color;
    }
    
    public boolean hasColor() {
        return this.colorSensor.getProximity() >= ShooterConstants.MIN_COLOR_SENSOR_PROXIMITY;
    }
}
