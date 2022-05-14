package frc.robot.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SensorWinchRetract extends CommandBase {
    private final ClimberMotorsSubsystem climberMotorsSubsystem;
    private boolean isBarDetected;
    private Timer timer;

    public SensorWinchRetract(ClimberMotorsSubsystem climberMotorsSubsystem) {
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        addRequirements(this.climberMotorsSubsystem);
    }

    @Override
    public void initialize() {
        this.climberMotorsSubsystem.setWinchSpeed(-1);
        this.isBarDetected = false;
        this.timer = new Timer();
        this.timer.start();
    }

    @Override
    public void execute() {
        if (!this.climberMotorsSubsystem.getClimberSensor()) {
            this.isBarDetected = true;
        }

        if (this.isBarDetected) {
            this.climberMotorsSubsystem.setWinchSpeed(-0.6);
        } else {
            this.climberMotorsSubsystem.setWinchSpeed(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.climberMotorsSubsystem.setWinchSpeed(0);
        this.climberMotorsSubsystem.getWinchEncoder().setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return this.climberMotorsSubsystem.hitRearLimitSwitch() || this.timer.hasElapsed(5);
    }
}
