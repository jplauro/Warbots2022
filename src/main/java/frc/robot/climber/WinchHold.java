package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WinchHold extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private final int endFrame;
    private double frames, holdCount;

    private final double winchHoldDiffConst = 0.3;

    public WinchHold(ClimberSubsystem climberSubsystem, double holdCount, int endFrame) {
        this.climberSubsystem = climberSubsystem;
        this.holdCount = holdCount;
        this.endFrame = endFrame;
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        double pos = this.climberSubsystem.getWinchPosition();
        double speed = this.winchHoldDiffConst * (this.holdCount - pos);
        this.climberSubsystem.setWinchSpeed(speed);
        this.frames++;
    }

    @Override
    public void end(boolean interrupted) {
        this.climberSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return this.frames >= this.endFrame;
    }
}
