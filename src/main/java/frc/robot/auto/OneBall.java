package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drive.DriveDirection;
import frc.robot.drive.DriveDistance;
import frc.robot.drive.Drivetrain;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.ActivateFiringPins;
import frc.robot.shooter.FiringPins;
import frc.robot.shooter.LimelightSpinUp;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.turret.CalibrateTurret;
import frc.robot.turret.TurretSubsystem;

public class OneBall extends SequentialCommandGroup {
    private final double FIRST_SHOT_DISTANCE = 2; // Meters

    public OneBall(Drivetrain drivetrain, TurretSubsystem turretSubsystem,
    ShooterSubsystem shooterSubsystem, FiringPins firingPins, IntakeSubsystem intakeSubsystem) {
        addRequirements(drivetrain, turretSubsystem, shooterSubsystem, firingPins, intakeSubsystem);
        addCommands(
            parallel(
                new ConditionalCommand(new InstantCommand(),
                    new CalibrateTurret(turretSubsystem), turretSubsystem::getIsCalibrated),
                new DriveDistance(drivetrain, this.FIRST_SHOT_DISTANCE, DriveDirection.FORWARD)
            ),
            new WaitCommand(1),
            new LimelightSpinUp(shooterSubsystem),
            new ActivateFiringPins(firingPins, intakeSubsystem)
        );
    }
}
