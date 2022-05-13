package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Drive.Direction;
import frc.robot.Drive.DriveDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ZeroTurnTable;

public class Taxi extends ParallelCommandGroup {
    private final double TAXI_DISTANCE = 2; // In meters

    public Taxi(Drivetrain drivetrain, Intake intake, LazySusanSubsystem lazySusanSubsystem) {
        addRequirements(drivetrain, intake, lazySusanSubsystem);
        addCommands(
            new ConditionalCommand(new ZeroTurnTable(lazySusanSubsystem),
                    new InstantCommand(), lazySusanSubsystem::getIsCal),
            new DriveDistance(drivetrain, this.TAXI_DISTANCE, Direction.FORWARD)
        );
    }
}
