package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ControlWinch.WinchMotion;
import frc.robot.Intake.IntakeSubsystem;

public class RaiseAndGrab extends SequentialCommandGroup {
    private final int RAISE_HOOKS_FRAMES = 100;

    public RaiseAndGrab(ClimberSubsystem climberSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(climberSubsystem, intakeSubsystem);
        addCommands(
            parallel(
                new ControlWinch(climberSubsystem, intakeSubsystem, 
                ClimberConstants.WINCH_LIMIT_MAX / 2, WinchMotion.RETRACT),
                new ControlPistons(climberSubsystem, PistonMotion.LOWER)
            ),
            new InstantCommand(() -> climberSubsystem.setHangingSolenoid(true)).withTimeout(0.3), // Lower hooks for 15 frames
            new SensorWinchRetract(climberSubsystem),
            parallel(
                new InstantCommand(() -> climberSubsystem.setHangingSolenoid(false)).withTimeout(0.3), // Raise hooks for 15 frames
                new WinchHold(climberSubsystem, -5, this.RAISE_HOOKS_FRAMES)
            )
        );
    }
}
