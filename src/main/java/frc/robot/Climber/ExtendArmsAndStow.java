package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ControlWinch.WinchMotion;
import frc.robot.Loader.Intake;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    public ExtendArmsAndStow(WinchSubsystem winchSubsystem, 
    ClimberSubsystem climberSubsystem, Intake intake) {
        addRequirements(winchSubsystem, climberSubsystem, intake);        
        addCommands(
            new ControlPistons(climberSubsystem, 0.1, PistonMotion.RAISE), // Bump the arms up slightly
            new ControlWinch(winchSubsystem, intake, 
            ClimberConstants.WINCH_LIMIT_MAX / 2, WinchMotion.EXTEND),
            new ControlPistons(climberSubsystem, 0.1, PistonMotion.LOWER),
            new ControlWinch(winchSubsystem, intake, 
            ClimberConstants.WINCH_LIMIT_MAX / 2, WinchMotion.EXTEND)
        );
    }
}