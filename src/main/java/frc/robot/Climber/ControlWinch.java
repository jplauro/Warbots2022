package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake.IntakeSubsystem;

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

    private final WinchSubsystem winchSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final boolean isExtending;
    private final double speed;
    private double deltaCounts, targetCounts;

    public ControlWinch(WinchSubsystem winchSubsystem, 
    IntakeSubsystem intakeSubsystem, double deltaCounts, WinchMotion motion) {
        this.winchSubsystem = winchSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.deltaCounts = deltaCounts;
        this.isExtending = motion == WinchMotion.EXTEND;
        this.speed = motion.get();
        addRequirements(this.winchSubsystem);
    }

    @Override
    public void initialize() {
        if (this.isExtending) {
            this.intakeSubsystem.extendIntakeArms();
        }

        this.targetCounts = this.winchSubsystem.getWinchPosition() 
        + this.deltaCounts * Math.signum(this.speed); // Subtract if the speed is negative

        this.winchSubsystem.setWinchSpeed(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        if (this.isExtending) {
            this.intakeSubsystem.floatIntakeArms();
        }

        this.winchSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (this.isExtending) {
            return this.winchSubsystem.getWinchPosition() >= 
            Math.min(this.targetCounts, ClimberConstants.WINCH_LIMIT_MAX);
        } else {
            return this.winchSubsystem.getWinchPosition() <= 
            Math.max(this.targetCounts, ClimberConstants.WINCH_LIMIT_MIN);
        }
    }
}
