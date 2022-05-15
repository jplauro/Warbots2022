package frc.robot.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LedMode;

public class TankDriveAiming extends CommandBase {
    private final Drivetrain drivetrain;
    private final TurretSubsystem turretSubsystem;
    private int frames, maxFrames;

    private final double tankDriveAimDiffConst = 0.014;

    public TankDriveAiming(Drivetrain drivetrain, TurretSubsystem turretSubsystem, int maxFrames) {
        this.drivetrain = drivetrain;
        this.turretSubsystem = turretSubsystem;
        this.maxFrames = maxFrames;
        addRequirements(this.drivetrain, this.turretSubsystem);
    }

    public TankDriveAiming(Drivetrain drivetrain, TurretSubsystem turretSubsystem) {
        this(drivetrain, turretSubsystem, -1);
    }

    @Override
    public void initialize() {
        this.frames = 0;
        Limelight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        this.frames++;
        double x = Limelight.getTX() + this.turretSubsystem.getOffsetDegrees();
        double speed = x * this.tankDriveAimDiffConst;
        this.drivetrain.tankDriveSet(speed, -speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.tankDriveSet(0, 0);
        Limelight.setLedMode(LedMode.OFF);
    }

    @Override
    public boolean isFinished() {
        return this.maxFrames > -1 ? this.frames >= this.maxFrames : false;
    }
}
