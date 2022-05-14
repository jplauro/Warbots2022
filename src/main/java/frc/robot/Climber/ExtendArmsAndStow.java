package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ControlWinch.WinchMotion;
import frc.robot.Loader.Intake;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    public ExtendArmsAndStow(ClimberMotorsSubsystem climberMotorsSubsystem, 
    ClimberSubsystem climberSubsystem, Intake intake) {
        addRequirements(climberMotorsSubsystem, climberSubsystem, intake);        
        addCommands(
            new ControlPistons(climberSubsystem, 0.1, PistonMotion.RAISE), // Bump the arms up slightly
            new ControlWinch(climberMotorsSubsystem, intake, 
            Constants.winchMaxLimit / 2.0, WinchMotion.EXTEND),
            new ControlPistons(climberSubsystem, 0.1, PistonMotion.LOWER),
            new ControlWinch(climberMotorsSubsystem, intake, 
            Constants.winchMaxLimit / 2.0, WinchMotion.EXTEND)
        );
    }
}