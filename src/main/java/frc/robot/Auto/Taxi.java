package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.drive.DriveDirection;
import frc.robot.drive.DriveDistance;
import frc.robot.drive.Drivetrain;
import frc.robot.turret.CalibrateTurret;
import frc.robot.turret.TurretSubsystem;

public class Taxi extends ParallelCommandGroup {
    private final double TAXI_DISTANCE = 2; // In meters

    public Taxi(Drivetrain drivetrain, TurretSubsystem turretSubsystem) {
        addCommands(
            new ConditionalCommand(new InstantCommand(),
                new CalibrateTurret(turretSubsystem), turretSubsystem::getIsCalibrated),
            new DriveDistance(drivetrain, this.TAXI_DISTANCE, DriveDirection.FORWARD)
        );
    }
}
