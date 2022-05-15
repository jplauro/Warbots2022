package frc.robot.Turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.RobotMath;

public class ManualAimingPID extends CommandBase {
    private final TurretSubsystem turretSubsystem;

    public ManualAimingPID(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void execute() {
        double input = RobotMath.deadZone(ControlBoard.manualAimingJoystick.getAsDouble(), 0.1);

        if (Math.abs(input) > 0) {
            this.turretSubsystem.setTurretPosition(this.turretSubsystem.getDesiredRotation()
            .minus(Rotation2d.fromDegrees(input * TurretConstants.MANUAL_AIMING_MOD_SPEED)));
        }
    }
}
