package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTurnTable extends CommandBase {
    private LazySusanSubsystem lazySusanSubsystem;
    private double targetRotation;
    private Timer timer;

    public ZeroTurnTable(LazySusanSubsystem lazySusanSubsystem) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        addRequirements(this.lazySusanSubsystem);
    }

    @Override
    public void initialize() {
        this.lazySusanSubsystem.setIsGyroLocking(false);
        this.lazySusanSubsystem.setIsHubTracking(false);
        this.lazySusanSubsystem.setSmartCurrentLimit(5);
        this.targetRotation = this.lazySusanSubsystem.getRotationDegrees() - 189.5;
        this.lazySusanSubsystem.setModSpeed(0.17); // 0.4
        this.lazySusanSubsystem.setTurretPositionDegrees(this.targetRotation);
        this.lazySusanSubsystem.setIsCal(false);
        this.timer = new Timer();
    }

    @Override
    public boolean isFinished() {
        if (this.lazySusanSubsystem.atTurretPosition()) {
            this.timer.start();
        }

        if (this.timer.hasElapsed(1)) {
            return true;
        }

        if (this.lazySusanSubsystem.islimitSwitchPressed()) {
            this.lazySusanSubsystem.setHomePosition();
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.lazySusanSubsystem.setModSpeed(1);
        this.lazySusanSubsystem.setSmartCurrentLimit(18);
        this.lazySusanSubsystem.setIsGyroLocking(true);
        this.lazySusanSubsystem.setIsHubTracking(true);
    }
}
