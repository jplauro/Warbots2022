package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drive.DriveDirection;
import frc.robot.Drive.DriveDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.ZeroTurnTable;

public class OneBall extends SequentialCommandGroup {
    private final double FIRST_SHOT_DISTANCE = 2; // In meters

    public OneBall(Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem,
    ShooterSubsystem shooterSubsystem, FiringPins firingPins, IntakeSubsystem intakeSubsystem) {
        addRequirements(drivetrain, lazySusanSubsystem, shooterSubsystem, firingPins, intakeSubsystem);
        addCommands(
            parallel(
                new ConditionalCommand(new ZeroTurnTable(lazySusanSubsystem), 
                    new InstantCommand(), lazySusanSubsystem::getIsCal),
                new DriveDistance(drivetrain, this.FIRST_SHOT_DISTANCE, DriveDirection.FORWARD)
            ),
            new WaitCommand(1),
            new LimelightSpinUp(shooterSubsystem),
            new ActivateFiringPins(firingPins, intakeSubsystem)
        );
    }
}
