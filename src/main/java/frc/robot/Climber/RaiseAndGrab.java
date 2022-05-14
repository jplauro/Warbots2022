package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ControlWinch.WinchMotion;
import frc.robot.Loader.Intake;

public class RaiseAndGrab extends SequentialCommandGroup {
    private final int RAISE_HOOKS_FRAMES = 100;

    public RaiseAndGrab(ClimberMotorsSubsystem climberMotorsSubsystem, 
    ClimberSubsystem climberSubsystem, Intake intake) {
        addRequirements(climberMotorsSubsystem, climberSubsystem, intake);
        addCommands(
            parallel(
                new ControlWinch(climberMotorsSubsystem, intake, 
                Constants.winchMaxLimit / 2.0, WinchMotion.RETRACT),
                new ControlPistons(climberSubsystem, PistonMotion.LOWER)
            ),
            new InstantCommand(() -> climberSubsystem.setHangingSolenoid(true)).withTimeout(0.3), // Lower hooks for 15 frames
            new SensorWinchRetract(climberMotorsSubsystem),
            parallel(
                new InstantCommand(() -> climberSubsystem.setHangingSolenoid(false)).withTimeout(0.3), // Raise hooks for 15 frames
                new WinchHold(climberMotorsSubsystem, -5, this.RAISE_HOOKS_FRAMES)
            )
        );
    }
}
