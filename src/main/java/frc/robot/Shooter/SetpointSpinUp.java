package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.Limelight;
import frc.robot.Util.Limelight.LedMode;

public class SetpointSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;
    protected double offsetY;
    private Timer timer;

    public SetpointSpinUp(ShooterSubsystem shooterSubsystem, double offsetY) {

        this.shooterSubsystem = shooterSubsystem;
        this.offsetY = offsetY;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetRPM(Constants.rpmMap.get(offsetY));
        ControlBoard.setOperatorRumble(getWithinTolerance());
    }

    private boolean getWithinTolerance() {
        return ShooterMath.withinTolerance(
            this.shooterSubsystem.getRPM(), 
            this.shooterSubsystem.getTargetRPM(), 
            Constants.shooterVibrationTolerance);
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupt) {
        shooterSubsystem.setTargetRPM(0);
        Limelight.setLedMode(LedMode.OFF);
        ControlBoard.setOperatorRumble(false);
    }
}