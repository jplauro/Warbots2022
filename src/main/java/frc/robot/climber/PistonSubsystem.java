package frc.robot.climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PistonSubsystem extends SubsystemBase {
    private final Solenoid armsSolenoid;
    private final Solenoid hooksSolenoid;

    public PistonSubsystem() {
        this.armsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.ARMS_SOLENOID_ID);
        this.hooksSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.HOOKS_SOLENOID_ID);
    }

    public boolean getArmsSolenoid() {
        return this.armsSolenoid.get();
    }

    public boolean getHooksSolenoid() {
        return this.hooksSolenoid.get();
    }

    public void setArmsSolenoid(boolean state) {
        this.armsSolenoid.set(state);
    }

    public void setHooksSolenoid(boolean state) {
        this.hooksSolenoid.set(state);
    }
}
