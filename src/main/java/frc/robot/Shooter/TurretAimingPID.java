package frc.robot.Shooter;

import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class TurretAimingPID extends CommandBase {
    protected LazySusanSubsystem lazySusanSubsystem;
    protected Supplier<Pose2d> robotbase;

    public TurretAimingPID(LazySusanSubsystem lazySusanSubsystem, Supplier<Pose2d> robotbase) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.robotbase = robotbase;
        addRequirements(lazySusanSubsystem);
    }

    @Override
    public void initialize() {
        LimeLight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        double x = LimeLight.getTX(), 
            distance = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, LimeLight.getTY(), Constants.limelightHeight, Constants.hubHeight);
        if (LimeLight.hasTarget()) {
            boolean inRange = LimeLight.inRange();
            lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(x)));
            ControlBoard.setOperatorRumble(!inRange);
            ControlBoard.setDriverRumble(!inRange);
        } else {
            ControlBoard.setOperatorRumble(true);
            ControlBoard.setDriverRumble(true);
        }
        SmartDashboard.putNumber("LimeLight Distance", distance);
        SmartDashboard.putNumber("LimeLight TY", LimeLight.getTY()); //TODO: Remove debug data


        var pose = new Pose2d(this.robotbase.get().getTranslation(),  robotbase.get().getRotation().plus(lazySusanSubsystem.getRotation()));
        var testpos = pose.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-LimeLight.getTX()))).transformBy(
            new Transform2d(new Translation2d(distance, 0), Rotation2d.fromDegrees(0)));
        // asdf.robotFieldWidget.getObject("hub-target").setPose(testpos);;
    }

    @Override
    public void end(boolean interrupt) {
        LimeLight.setLedMode(LedMode.OFF);
        ControlBoard.setOperatorRumble(false);
        ControlBoard.setDriverRumble(false);
    }
}
