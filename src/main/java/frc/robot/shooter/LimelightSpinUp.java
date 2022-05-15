package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LedMode;

public class LimelightSpinUp extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    public LimelightSpinUp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        Limelight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        Limelight.setLedMode(LedMode.ON); // TODO: Less jank
        double targetRPM = ShooterConstants.RPM_MAP.get(Limelight.getTY());

        if (Limelight.hasTarget()) {
            this.shooterSubsystem.setTargetRPM(targetRPM + this.shooterSubsystem.getOffsetSpeed());
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.stopMotors();
        Limelight.setLedMode(LedMode.OFF);
    }
}
