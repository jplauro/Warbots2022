package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.drive.DriveDirection;
import frc.robot.drive.DriveDistance;
import frc.robot.drive.Drivetrain;
import frc.robot.intake.ControlIntake;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.ControlIntake.IntakeMotion;
import frc.robot.shooter.ActivateFiringPins;
import frc.robot.shooter.FiringPins;
import frc.robot.shooter.LimelightSpinUp;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.turret.CalibrateTurret;
import frc.robot.turret.TurretAimingPID;
import frc.robot.turret.TurretSubsystem;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LedMode;

public class TwoBalls extends SequentialCommandGroup {
    private final double FIRST_SHOT_DISTANCE = 0.5; // Meters
    private final double SECOND_SHOT_DISTANCE = 1.3; // Meters
    private final double TAXI_DISTANCE = 1.6; // Meters
    private final int INTAKE_FRAMES = 100;

    public TwoBalls(RobotContainer robotContainer, Drivetrain drivetrain, TurretSubsystem turretSubsystem,
    ShooterSubsystem shooterSubsystem, FiringPins firingPins, IntakeSubsystem intakeSubsystem) {
        addRequirements(drivetrain, turretSubsystem, shooterSubsystem, firingPins, intakeSubsystem);
        addCommands(
            parallel(
                new LimelightSpinUp(shooterSubsystem),
                sequence(
                    parallel(
                        // Only calibrate if not already calibrated
                        new ConditionalCommand(new InstantCommand(),
                            new CalibrateTurret(turretSubsystem), turretSubsystem::getIsCalibrated),
                        new DriveDistance(drivetrain, this.FIRST_SHOT_DISTANCE, DriveDirection.FORWARD)
                    ),
                    new WaitCommand(2),
                    new TurretAimingPID(turretSubsystem, robotContainer.getRobotField(), 
                        drivetrain::getPose, 100, false),
                    new ActivateFiringPins(firingPins, intakeSubsystem),
                    parallel(
                        new ControlIntake(intakeSubsystem, IntakeMotion.INTAKE, this.INTAKE_FRAMES),
                        new DriveDistance(drivetrain, this.SECOND_SHOT_DISTANCE, DriveDirection.FORWARD)
                    ),
                    new DriveDistance(drivetrain, this.SECOND_SHOT_DISTANCE, DriveDirection.BACKWARD),
                    new TurretAimingPID(turretSubsystem, robotContainer.getRobotField(), 
                        drivetrain::getPose, 100, false),
                    new ActivateFiringPins(firingPins, intakeSubsystem),
                    new WaitCommand(1),
                    new DriveDistance(drivetrain, this.TAXI_DISTANCE, DriveDirection.FORWARD)
                )
            ),
            new InstantCommand(() -> Limelight.setLedMode(LedMode.OFF))
        );
    }
}
