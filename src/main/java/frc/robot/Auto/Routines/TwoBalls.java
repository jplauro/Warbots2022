package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Drive.Direction;
import frc.robot.Drive.DriveDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.AutoLoad;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.TurretAimingPID;
import frc.robot.Shooter.ZeroTurnTable;
import frc.robot.Util.Limelight;
import frc.robot.Util.Limelight.LedMode;

public class TwoBalls extends SequentialCommandGroup {
    // All distances are in meters
    private double FIRST_SHOT_DISTANCE = 0.5;
    private double SECOND_SHOT_DISTANCE = 1.3;
    private double TAXI_DISTANCE = 1.6;

    public TwoBalls(RobotContainer robotContainer, Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem,
    ShooterSubsystem shooterSubsystem, FiringPins firingPins, Intake intake) {
        addCommands(
            parallel(
                new LimelightSpinUp(shooterSubsystem),
                sequence(
                    parallel(
                        // Only calibrate if not already calibrated
                        new ConditionalCommand(new ZeroTurnTable(lazySusanSubsystem),
                            new InstantCommand(), lazySusanSubsystem::getIsCal),
                        new DriveDistance(drivetrain, FIRST_SHOT_DISTANCE, Direction.FORWARD)
                    ),
                    new WaitCommand(2),
                    new TurretAimingPID(lazySusanSubsystem, robotContainer.getRobotField(), 
                        drivetrain::getPose, 100, false),
                    new ActivateFiringPins(firingPins, intake),
                    parallel(
                        new AutoLoad(intake),
                        new DriveDistance(drivetrain, SECOND_SHOT_DISTANCE, Direction.FORWARD)
                    ),
                    new DriveDistance(drivetrain, SECOND_SHOT_DISTANCE, Direction.BACKWARD),
                    new TurretAimingPID(lazySusanSubsystem, robotContainer.getRobotField(), 
                        drivetrain::getPose, 100, false),
                    new ActivateFiringPins(firingPins, intake),
                    new WaitCommand(1),
                    new DriveDistance(drivetrain, TAXI_DISTANCE, Direction.FORWARD)
                )
            ),
            new InstantCommand(() -> Limelight.setLedMode(LedMode.OFF))
        );
    }
}
