package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class RaiseAndGrab extends SequentialCommandGroup {
    private final int RAISE_HOOKS_FRAMES = 100;

    public RaiseAndGrab(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
        addRequirements(climberMotorsSubsystem, climberSubsystem);
        addCommands(
            parallel(
                new WinchRetract(climberMotorsSubsystem, Constants.winchMaxLimit / 2.0),
                new LowerPistons(climberSubsystem)),
            new LowerHooks(climberSubsystem),
            new SensorWinchRetract(climberMotorsSubsystem, climberSubsystem),
            parallel(
                new RaiseHooks(climberSubsystem, climberMotorsSubsystem),
                new WinchHold(climberMotorsSubsystem, -5, this.RAISE_HOOKS_FRAMES))
        );
    }
}
