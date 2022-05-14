package frc.robot.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
    private Drivetrain drivetrain;
    private double distance;
    private double direction;
    private Pose2d initPose;

    public DriveDistance(Drivetrain drivetrain, double distance, DriveDirection direction) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.direction = direction.get();
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        this.initPose = this.drivetrain.getPose();
    }

    @Override
    public void execute() {
        this.drivetrain.arcadeDriveSet(this.direction * DriveConstants.AUTO_SPEED, 0);
    }

    public boolean withinBounds() {
        return this.getDisplacement() > this.distance;
    }

    private double getDisplacement() {
        return this.initPose.getTranslation().getDistance(this.drivetrain.getPose().getTranslation());
    }

    @Override
    public boolean isFinished() {
        return withinBounds();
    }

    @Override
    public void end(boolean interrupt) {
        this.drivetrain.arcadeDriveSet(0, 0);
    }
}
