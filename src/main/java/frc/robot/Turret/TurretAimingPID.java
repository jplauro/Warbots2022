package frc.robot.turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LedMode;

public class TurretAimingPID extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private final Field2d robotField;
    private final Supplier<Pose2d> robotBase;
    private final boolean endOnSuccessfulTracking;

    private Pose2d prevHubPosition;
    private int frames, maxFrames;

    public TurretAimingPID(TurretSubsystem turretSubsystem, Field2d robotField,
    Supplier<Pose2d> robotbase, int maxFrames, boolean endOnSuccessfulTracking) {
        this.turretSubsystem = turretSubsystem;
        this.robotField = robotField;
        this.robotBase = robotbase;
        this.maxFrames = maxFrames;
        this.endOnSuccessfulTracking = endOnSuccessfulTracking;
        addRequirements(this.turretSubsystem);
    }

    public TurretAimingPID(TurretSubsystem turretSubsystem,
    Field2d robotFieldWidget, Supplier<Pose2d> robotbase) {
        this(turretSubsystem, robotFieldWidget, robotbase, -1, false);
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

        // Don't track using the Limelight if the turret is still turning in the opposite direction
        if (Math.abs(this.turretSubsystem.getRawSetpoint() - this.turretSubsystem.getRotationDegrees()) <= 180) {
            if (Limelight.hasTarget()) {
                if (this.turretSubsystem.getIsGyroLocking()) {
                    this.turretSubsystem.setTurretPosition(this.robotBase.get().getRotation()
                    .plus(this.turretSubsystem.getRotation()).minus(Rotation2d.fromDegrees(x)));
                } else {
                    this.turretSubsystem.setTurretPosition(this.turretSubsystem.getRotation()
                    .minus(Rotation2d.fromDegrees(x)));
                }

                this.prevHubPosition = this.calculateHubPosition(this.getLocalPose());
            } else if (this.turretSubsystem.getIsHubTracking() && this.prevHubPosition != null) {
                double deltaX = this.calculateHubDeltaX(this.robotBase.get(), this.prevHubPosition);

                if (this.turretSubsystem.getIsGyroLocking()) {
                    this.turretSubsystem.setTurretPosition(Rotation2d.fromDegrees(deltaX));
                } else {
                    this.turretSubsystem.setTurretPosition(this.robotBase.get().getRotation()
                    .times(-1).plus(Rotation2d.fromDegrees(deltaX)));
                }
            }
        }
    }

    private double getDistanceInMeters() {
        return (Constants.HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT) 
        / Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + Limelight.getTY()));
    }

    private Pose2d getLocalPose() {
        return new Pose2d(
            this.robotBase.get().getTranslation(),
            this.robotBase.get().getRotation().plus(this.turretSubsystem.getRotation())
        );
    }

    private Pose2d calculateHubPosition(Pose2d robotPose) {
        Pose2d hubPosition = robotPose.transformBy(
            new Transform2d(new Translation2d(this.getDistanceInMeters(), 0), 
            Rotation2d.fromDegrees(-Limelight.getTX()))
        );
        this.robotField.getObject("hub-target").setPose(hubPosition);
        return hubPosition;
    }

    private double calculateHubDeltaX(Pose2d robotPose, Pose2d hubPose) {
        Translation2d relativePosition = hubPose.getTranslation().minus(robotPose.getTranslation());
        return Math.toDegrees(Math.atan2(relativePosition.getY(), relativePosition.getX()));
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.setLedMode(LedMode.OFF);
    }

    @Override
    public boolean isFinished() {
        if (this.endOnSuccessfulTracking && this.frames < this.maxFrames) {
            return Math.abs(Limelight.getTX()) < TurretConstants.LIMELIGHT_TOLERANCE;
        } else {
            return this.maxFrames > -1 ? this.frames >= this.maxFrames : false;
        }
    }
}
