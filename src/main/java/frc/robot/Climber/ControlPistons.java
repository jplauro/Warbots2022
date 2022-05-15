package frc.robot.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlPistons extends CommandBase {
    public enum PistonMotion {
        LOWER(false),
        RAISE(true);
    
        private final boolean motion;
    
        private PistonMotion(boolean motion) {
            this.motion = motion;
        }
    
        protected boolean get() {
            return this.motion;
        }
    }

    private final PistonSubsystem pistonSubsystem;
    private final double percent;
    private final boolean motion;
    private int frames, targetFrames;

    public ControlPistons(PistonSubsystem pistonSubsystem, double percent, PistonMotion motion) {
        this.pistonSubsystem = pistonSubsystem;
        this.percent = MathUtil.clamp(percent, 0, 1);
        this.motion = motion.get();
        addRequirements(this.pistonSubsystem);
    }

    public ControlPistons(PistonSubsystem pistonSubsystem, PistonMotion motion) {
        this(pistonSubsystem, 1, motion);
    }

    @Override
    public void initialize() {
        this.frames = 0;
        this.targetFrames = (int) Math.round(this.percent * ClimberConstants.PISTON_FRAMES_MAX);
        this.pistonSubsystem.setArmsSolenoid(this.motion);
    }

    @Override
    public void execute() {
        this.frames++;
    }

    @Override
    public boolean isFinished() {
        return this.frames >= this.targetFrames;
    }
}
