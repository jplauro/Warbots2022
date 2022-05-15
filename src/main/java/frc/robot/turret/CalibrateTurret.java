package frc.robot.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateTurret extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private double targetRotation;
    private Timer timer;

    public CalibrateTurret(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void initialize() {
        this.turretSubsystem.setSmartCurrentLimit(5);
        this.targetRotation = this.turretSubsystem.getRotationDegrees() - TurretConstants.LIMIT_SWITCH_POSITION;
        this.turretSubsystem.setModSpeed(TurretConstants.CALIBRATION_MOD_SPEED);
        this.turretSubsystem.setTurretPositionDegrees(this.targetRotation);
        this.timer = new Timer();
    }

    @Override
    public void end(boolean interrupted) {
        this.turretSubsystem.setModSpeed(TurretConstants.MOD_SPEED);
        this.turretSubsystem.setSmartCurrentLimit(TurretConstants.CURRENT_LIMIT);
        this.turretSubsystem.setIsGyroLocking(true);
        this.turretSubsystem.setIsHubTracking(true);
    }

    @Override
    public boolean isFinished() {
        if (this.turretSubsystem.atTurretPosition()) {
            this.timer.start();
        }

        if (this.turretSubsystem.getLimitSwitch()) {
            this.turretSubsystem.setHomePosition();
            return true;
        }

        // Ensure that the motor is not stalling or being damaged
        // This returns from the command if the switch is not hit
        if (this.timer.hasElapsed(1)) {
            return true;
        }

        return false;
    }
}
