package frc.robot.Turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.RobotMath;

public class ManualAimingPID extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private final XboxController operator;

    public ManualAimingPID(TurretSubsystem turretSubsystem, XboxController operator) {
        this.turretSubsystem = turretSubsystem;
        this.operator = operator;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void execute() {
        double input = RobotMath.deadZone(this.operator.getLeftX(), 0.1, 0);

        if (Math.abs(input) > 0) {
            turretSubsystem.setTurretPosition(turretSubsystem.getDesiredRotation()
            .minus(Rotation2d.fromDegrees(input * TurretConstants.MANUAL_AIMING_MOD_SPEED)));
        }
    }
}
