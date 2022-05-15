package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ControlWinch.WinchMotion;
import frc.robot.Intake.IntakeSubsystem;

public class ExtendArms extends SequentialCommandGroup {
    public ExtendArms(ClimberSubsystem climberSubsystem, 
    PistonSubsystem pistonSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(climberSubsystem, pistonSubsystem, intakeSubsystem);        
        addCommands(
            new ControlPistons(pistonSubsystem, 0.1, PistonMotion.RAISE), // Bump the arms up slightly
            new ControlWinch(climberSubsystem, intakeSubsystem, 
                ClimberConstants.WINCH_LIMIT_MAX / 2.0, WinchMotion.EXTEND),
            new ControlPistons(pistonSubsystem, 0.1, PistonMotion.LOWER),
            new ControlWinch(climberSubsystem, intakeSubsystem, 
                ClimberConstants.WINCH_LIMIT_MAX / 2.0, WinchMotion.EXTEND)
        );
    }
}
