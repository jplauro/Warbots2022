package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrees extends CommandBase {
    private final Drivetrain drivetrain;
    private double deltaDegrees, targetDegrees;

    private final double tolerance = 3, maxSpeed = 0.8, turnDiffConst = 0.008;

    public TurnDegrees(Drivetrain drivetrain, double deltaDegrees) {
        this.drivetrain = drivetrain;
        this.deltaDegrees = deltaDegrees;
        addRequirements(this.drivetrain);
    }

    public void initialize() {
        this.targetDegrees = this.drivetrain.getHeading() + this.deltaDegrees;
    }

    public void execute() {
        double speed = this.turnDiffConst * (this.targetDegrees - this.drivetrain.getHeading());
        speed = Math.signum(speed) * Math.min(Math.abs(speed), this.maxSpeed);
        this.drivetrain.tankDriveSet(-speed, speed);
    }

    public boolean isFinished() {
        double angle = this.drivetrain.getHeading();
        return angle > this.targetDegrees - this.tolerance && 
            angle < this.targetDegrees + this.tolerance;
    }
}
