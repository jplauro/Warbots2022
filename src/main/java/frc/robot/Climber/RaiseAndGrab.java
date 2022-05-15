package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ControlWinch.WinchMotion;
import frc.robot.Intake.IntakeSubsystem;

public class RaiseAndGrab extends SequentialCommandGroup {
    private final int RAISE_HOOKS_FRAMES = 100;

    public RaiseAndGrab(ClimberSubsystem climberSubsystem, 
    PistonSubsystem pistonSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(climberSubsystem, pistonSubsystem, intakeSubsystem);
        addCommands(
            parallel(
                new ControlWinch(climberSubsystem, intakeSubsystem, 
                    ClimberConstants.WINCH_LIMIT_MAX / 2.0, WinchMotion.RETRACT),
                new ControlPistons(pistonSubsystem, PistonMotion.LOWER)
            ),
            new InstantCommand(() -> pistonSubsystem.setHooksSolenoid(true)),
            new WaitCommand(0.3), // Lower hooks for 15 frames
            new SensorWinchRetract(climberSubsystem),
            parallel(
                sequence(
                    new InstantCommand(() -> pistonSubsystem.setHooksSolenoid(false)),
                    new WaitCommand(0.3) // Raise hooks for 15 frames  
                ),
                new WinchHold(climberSubsystem, -5, this.RAISE_HOOKS_FRAMES)
            )
        );
    }
}
