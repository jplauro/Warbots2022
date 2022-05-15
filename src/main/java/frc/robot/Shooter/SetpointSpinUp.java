package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.Limelight;
import frc.robot.Util.Limelight.LedMode;

public class SetpointSpinUp extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double offsetY;
    private Timer timer;

    public SetpointSpinUp(ShooterSubsystem shooterSubsystem, double offsetY) {
        this.shooterSubsystem = shooterSubsystem;
        this.offsetY = offsetY;
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.timer = new Timer();
        this.timer.start();
    }

    @Override
    public void execute() {
        this.shooterSubsystem.setTargetRPM(ShooterConstants.RPM_MAP.get(this.offsetY));
        ControlBoard.setOperatorRumble(this.shooterSubsystem.isWithinTolerance());
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.setTargetRPM(0);
        ControlBoard.setOperatorRumble(false);
        Limelight.setLedMode(LedMode.OFF);
    }
}