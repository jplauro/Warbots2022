package frc.robot.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SensorWinchRetract extends CommandBase {
    private final WinchSubsystem winchSubsystem;
    private boolean isBarDetected;
    private Timer timer;

    public SensorWinchRetract(WinchSubsystem winchSubsystem) {
        this.winchSubsystem = winchSubsystem;
        addRequirements(this.winchSubsystem);
    }

    @Override
    public void initialize() {
        this.winchSubsystem.setWinchSpeed(-1);
        this.isBarDetected = false;
        this.timer = new Timer();
        this.timer.start();
    }

    @Override
    public void execute() {
        if (!this.winchSubsystem.getProximitySensor()) {
            this.isBarDetected = true;
        }

        if (this.isBarDetected) {
            this.winchSubsystem.setWinchSpeed(-0.6);
        } else {
            this.winchSubsystem.setWinchSpeed(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.winchSubsystem.setWinchSpeed(0);
        this.winchSubsystem.setWinchPosition(0);
    }

    @Override
    public boolean isFinished() {
        return this.winchSubsystem.getWinchLimitSwitch() || this.timer.hasElapsed(5);
    }
}
