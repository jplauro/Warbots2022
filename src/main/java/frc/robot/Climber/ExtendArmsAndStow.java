package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Loader.Intake;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    public ExtendArmsAndStow(ClimberMotorsSubsystem climberMotorsSubsystem, 
    ClimberSubsystem climberSubsystem, Intake intake) {
        addRequirements(climberMotorsSubsystem, climberSubsystem, intake);        
        addCommands(
            new RaisePistons(climberSubsystem, 0.1), // Bump the arms up slightly
            new WinchExtend(climberMotorsSubsystem, intake, Constants.winchMaxLimit / 2.0),
            new LowerPistons(climberSubsystem, 0.1),
            new WinchExtend(climberMotorsSubsystem, intake, Constants.winchMaxLimit / 2.0)
        );
    }
}