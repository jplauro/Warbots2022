package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveInDirection extends CommandBase {
    private final Drivetrain drivetrain;
    private final double direction;
    private final double speed;
    private final double targetFrames;
    private double frames;

    public DriveInDirection(Drivetrain drivetrain, DriveDirection direction, double speed, double targetFrames) {
        this.drivetrain = drivetrain;
        this.direction = direction.get();
        this.speed = speed;
        this.targetFrames = targetFrames;
        addRequirements(this.drivetrain);
    }

    public DriveInDirection(Drivetrain drivetrain, DriveDirection direction) {
        this(drivetrain, direction, 0.5, 60);
    }

    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        drivetrain.curvatureInput(this.direction * this.speed, 0, false);
        this.frames++;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setBrake(true);
    } 

    @Override
    public boolean isFinished() {
        return this.frames > this.targetFrames;
    }
}
