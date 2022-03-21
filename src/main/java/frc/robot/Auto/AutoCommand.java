package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drive.DriveForwardsEncoder;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.DirectTurretAuto;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Util.RobotContainer;
import frc.robot.Util.WaitFrames;

public class AutoCommand extends SequentialCommandGroup {
    FiringPins firingPins;
    ShooterSubsystem shooterSubsystem;
    LazySusanSubsystem lazySusanSubsystem;
    RobotContainer robotContainer;
    public AutoCommand(FiringPins fP, ShooterSubsystem sS, LazySusanSubsystem lSS, RobotContainer rC) {
        this.firingPins = fP;
        this.shooterSubsystem = sS;
        this.lazySusanSubsystem = lSS;
        this.robotContainer = rC;
        // addRequirements(loaderSubsystem, shooterSubsystem, lazySusanSubsystem);
            addCommands(
            //robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part1.wpilib.json"))),
                // new DriveForwards(robotContainer.getDriveTrain()),
                new DriveForwardsEncoder(robotContainer.getDriveTrain(), 2.7),

                //new TurnDegrees(robotContainer.getDriveTrain(), 180),

                new WaitFrames(150),

                new ActivateFiringPins(firingPins),

                new WaitFrames(150),

                // new DriveForwardsEncoder(robotContainer.getDriveTrain(), 0.3),

                // new WaitCommand(1),

                new ActivateFiringPins(firingPins),

                new WaitCommand(150)
                

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part2.wpilib.json"))),

                // new AutoShoot(loaderSubsystem),

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part3.wpilib.json"))),

                // new AutoShoot(loaderSubsystem),

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part4.wpilib.json"))),

                // new AutoShoot(loaderSubsystem)
                
                
            );
        
    }
}
