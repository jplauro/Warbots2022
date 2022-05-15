package frc.robot.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateTurret extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private double targetRotation;

    public CalibrateTurret(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void initialize() {
        this.turretSubsystem.setSmartCurrentLimit(5);
        this.targetRotation = this.turretSubsystem.getRotationDegrees() - TurretConstants.LIMIT_SWITCH_POSITION;
        this.turretSubsystem.setModSpeed(0.17);
        this.turretSubsystem.setTurretPositionDegrees(this.targetRotation);
    }

    @Override
    public void end(boolean interrupted) {
        this.turretSubsystem.setHomePosition();
        this.turretSubsystem.setModSpeed(TurretConstants.MOD_SPEED);
        this.turretSubsystem.setSmartCurrentLimit(TurretConstants.CURRENT_LIMIT);
        this.turretSubsystem.setIsGyroLocking(true);
        this.turretSubsystem.setIsHubTracking(true);
    }

    @Override
    public boolean isFinished() {
        return this.turretSubsystem.getLimitSwitch();
    }
}
