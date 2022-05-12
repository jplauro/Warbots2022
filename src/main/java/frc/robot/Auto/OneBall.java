package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drive.Direction;
import frc.robot.Drive.DriveDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.ZeroTurnTable;

public class OneBall extends SequentialCommandGroup {
    private double FIRST_SHOT_DISTANCE = 2; // In meters

    public OneBall(Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem,
    ShooterSubsystem shooterSubsystem, FiringPins firingPins, Intake intake) {
        addCommands(
            parallel(
                new ConditionalCommand(new ZeroTurnTable(lazySusanSubsystem), 
                    new InstantCommand(), lazySusanSubsystem::getIsCal),
                new DriveDistance(drivetrain, FIRST_SHOT_DISTANCE, Direction.FORWARD)
            ),
            new WaitCommand(1),
            new LimelightSpinUp(shooterSubsystem),
            new ActivateFiringPins(firingPins, intake)
        );
    }
}
