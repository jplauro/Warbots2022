package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Loader.Intake;

public class ControlWinch extends CommandBase {
    public enum WinchMotion {
        RETRACT(-1),
        EXTEND(0.75);
    
        private final double speed;
    
        private WinchMotion(double speed) {
            this.speed = speed;
        }
    
        protected double get() {
            return this.speed;
        }
    }

    private final ClimberMotorsSubsystem climberMotorsSubsystem;
    private final Intake intake;
    private final boolean isExtending;
    private final double speed;
    private double deltaCounts, targetCounts;

    public ControlWinch(ClimberMotorsSubsystem climberMotorsSubsystem, 
    Intake intake, double deltaCounts, WinchMotion motion) {
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.intake = intake;
        this.deltaCounts = deltaCounts;
        this.isExtending = motion == WinchMotion.EXTEND;
        this.speed = motion.get();
        addRequirements(climberMotorsSubsystem);
    }

    @Override
    public void initialize() {
        if (this.isExtending) {
            this.intake.extendIntakeArms();
        }

        this.targetCounts = this.climberMotorsSubsystem.getWinchPosition() 
        + this.deltaCounts * Math.signum(this.speed); // Subtract if the speed is negative

        this.climberMotorsSubsystem.setWinchSpeed(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        if (this.isExtending) {
            this.intake.floatIntakeArms();
        }

        this.climberMotorsSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (this.isExtending) {
            return climberMotorsSubsystem.getWinchPosition() >= 
            Math.min(this.targetCounts, Constants.winchMaxLimit);
        } else {
            return climberMotorsSubsystem.getWinchPosition() <= 
            Math.max(this.targetCounts, Constants.winchMinLimit);
        }
    }
}
