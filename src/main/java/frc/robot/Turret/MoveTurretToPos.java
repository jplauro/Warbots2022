package frc.robot.Turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveTurretToPos extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private double position = TurretConstants.STOWED_DEGREES;

    public MoveTurretToPos(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Turret/Test Pos", this.position);
    }

    @Override
    public void execute() {
        this.position = SmartDashboard.getNumber("Turret/Test Pos", this.position);
        this.turretSubsystem.setTurretPosition(Rotation2d.fromDegrees(this.position));   
    }
}
