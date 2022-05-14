package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class WinchHold extends CommandBase {
    private final WinchSubsystem winchSubsystem;
    private final int endFrame;
    private double frames, holdCount;

    public WinchHold(WinchSubsystem winchSubsystem, double holdCount, int endFrame) {
        this.winchSubsystem = winchSubsystem;
        this.holdCount = holdCount;
        this.endFrame = endFrame;
        addRequirements(this.winchSubsystem);
    }

    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        double pos = this.winchSubsystem.getWinchPosition();
        double speed = Constants.diffConstWinchHold * (this.holdCount - pos);
        this.winchSubsystem.setWinchSpeed(speed);
        this.frames++;
    }

    @Override
    public void end(boolean interrupted) {
        this.winchSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return this.frames >= this.endFrame;
    }
}
