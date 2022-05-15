package frc.robot.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestSetpointSpinUp extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private double testRPM;

    public TestSetpointSpinUp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.testRPM = 0;
        SmartDashboard.putNumber("Shooter/Test RPM", this.testRPM);
    }

    @Override
    public void execute() {
        this.testRPM = SmartDashboard.getNumber("Shooter/Test RPM", this.testRPM);
        this.shooterSubsystem.setTargetRPM(this.testRPM);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.setTargetRPM(0);
    }
}
