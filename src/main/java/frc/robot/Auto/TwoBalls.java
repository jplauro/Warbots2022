package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Drive.DriveDirection;
import frc.robot.Drive.DriveDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Intake.ControlIntake;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.ControlIntake.IntakeMotion;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Turret.TurretSubsystem;
import frc.robot.Turret.TurretAimingPID;
import frc.robot.Turret.CalibrateTurret;
import frc.robot.Util.Limelight;
import frc.robot.Util.Limelight.LedMode;

public class TwoBalls extends SequentialCommandGroup {
    // All distances are in meters
    private final double FIRST_SHOT_DISTANCE = 0.5;
    private final double SECOND_SHOT_DISTANCE = 1.3;
    private final double TAXI_DISTANCE = 1.6;
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
                        new ConditionalCommand(new CalibrateTurret(turretSubsystem),
                            new InstantCommand(), turretSubsystem::getIsCalibrated),
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
