package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Drive.DriveDirection;
import frc.robot.Drive.DriveDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Turret.CalibrateTurret;
import frc.robot.Turret.TurretSubsystem;

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
