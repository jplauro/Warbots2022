package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ControlWinch.WinchMotion;
import frc.robot.Intake.IntakeSubsystem;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    public ExtendArmsAndStow(WinchSubsystem winchSubsystem, 
    ClimberSubsystem climberSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(winchSubsystem, climberSubsystem, intakeSubsystem);        
        addCommands(
            new ControlPistons(climberSubsystem, 0.1, PistonMotion.RAISE), // Bump the arms up slightly
            new ControlWinch(winchSubsystem, intakeSubsystem, 
            ClimberConstants.WINCH_LIMIT_MAX / 2, WinchMotion.EXTEND),
            new ControlPistons(climberSubsystem, 0.1, PistonMotion.LOWER),
            new ControlWinch(winchSubsystem, intakeSubsystem, 
            ClimberConstants.WINCH_LIMIT_MAX / 2, WinchMotion.EXTEND)
        );
    }
}