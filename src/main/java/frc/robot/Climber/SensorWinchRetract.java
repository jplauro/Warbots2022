package frc.robot.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SensorWinchRetract extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private boolean isBarDetected;
    private Timer timer;

    public SensorWinchRetract(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        this.climberSubsystem.setWinchSpeed(-1);
        this.isBarDetected = false;
        this.timer = new Timer();
        this.timer.start();
    }

    @Override
    public void execute() {
        if (!this.climberSubsystem.getProximitySensor()) {
            this.isBarDetected = true;
        }

        if (this.isBarDetected) {
            this.climberSubsystem.setWinchSpeed(-0.6);
        } else {
            this.climberSubsystem.setWinchSpeed(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.climberSubsystem.setWinchSpeed(0);
        this.climberSubsystem.setWinchPosition(0);
    }

    @Override
    public boolean isFinished() {
        return this.climberSubsystem.getWinchLimitSwitch() || this.timer.hasElapsed(5);
    }
}
