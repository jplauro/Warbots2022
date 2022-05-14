package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class WinchHold extends CommandBase {
    private final ClimberMotorsSubsystem climberMotorsSubsystem;
    private final int endFrame;
    private double frames, holdCount;

    public WinchHold(ClimberMotorsSubsystem climberMotorsSubsystem, double holdCount, int endFrame) {
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.holdCount = holdCount;
        this.endFrame = endFrame;
        addRequirements(this.climberMotorsSubsystem);
    }

    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        double pos = this.climberMotorsSubsystem.getWinchPosition();
        double speed = Constants.diffConstWinchHold * (this.holdCount - pos);
        this.climberMotorsSubsystem.setWinchSpeed(speed);
        frames++;
    }

    @Override
    public void end(boolean interrupted) {
        this.climberMotorsSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return this.frames >= this.endFrame;
    }
}
