package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.Limelight;
import frc.robot.Util.Limelight.LedMode;

public class LimelightSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;

    public LimelightSpinUp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        Limelight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        Limelight.setLedMode(LedMode.ON); // TODO: Less jank
        double y = Limelight.getTY();
        // double distance = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        // double targetRPM = ShooterMath.metersToRPM(distance);

        //boolean hasTargetAndInRange = LimeLight.hasTarget() && Constants.rpmMap.isKeyInBounds(y);

        //ControlBoard.setOperatorHighFreqRumble(hasTargetAndInRange);
        
        // TODO: Use the below RPM value once the table is working
        double targetRPM = Constants.rpmMap.get(y);
        if(Limelight.hasTarget())
            this.shooterSubsystem.setTargetRPM(targetRPM+shooterSubsystem.getOffsetSpeed());//+100
        //ControlBoard.setOperatorLowFreqRumble(hasTargetAndInRange && this.getWithinTolerance());
    }

    // private boolean getWithinTolerance(){
    //     return ShooterMath.withinTolerance(
    //         this.shooterSubsystem.getRPM(), 
    //         this.shooterSubsystem.getTargetRPM(), 
    //         Constants.shooterVibrationTolerance);
    // }

    @Override
    public void end(boolean interrupt) {
        this.shooterSubsystem.stopMotors();
        Limelight.setLedMode(LedMode.OFF);
        // ControlBoard.setOperatorRumble(false);
    }
}