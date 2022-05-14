package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlIntake extends CommandBase {
    public enum IntakeMotion {
        EXTAKE(false),
        INTAKE(true);
    
        private final boolean motion;
    
        private IntakeMotion(boolean motion) {
            this.motion = motion;
        }
    
        protected boolean get() {
            return this.motion;
        }
    }

    private final IntakeSubsystem intakeSubsystem;
    private final boolean isIntaking;
    private int frames, maxFrames;

    public ControlIntake(IntakeSubsystem intakeSubsystem, IntakeMotion motion, int maxFrames) {   
        this.intakeSubsystem = intakeSubsystem;
        this.isIntaking = motion.get();
        this.maxFrames = maxFrames;
        addRequirements(this.intakeSubsystem);
    }

    public ControlIntake(IntakeSubsystem intakeSubsystem, IntakeMotion motion) {   
        this(intakeSubsystem, motion, -1);
    }

    @Override
    public void initialize() {
        this.frames = 0;

        if (this.isIntaking) {
            this.intakeSubsystem.enableInnerIntakeMotor();
            this.intakeSubsystem.enableIntakeArmsMotor();
            this.intakeSubsystem.extendIntakeArms();
        } else {
            this.intakeSubsystem.reverseInnerIntakeMotor();
            this.intakeSubsystem.disableIntakeArmsMotor();
            this.intakeSubsystem.retractIntakeArms();
        }
    }

    @Override
    public void execute() {
        this.frames++;

        if (this.isIntaking && this.frames > IntakeConstants.FRAMES_UNTIL_FLOAT) {
            this.intakeSubsystem.floatIntakeArms();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.disableInnerIntakeMotor();
        this.intakeSubsystem.disableIntakeArmsMotor();
        this.intakeSubsystem.retractIntakeArms();
    }

    @Override
    public boolean isFinished() {
        return this.maxFrames > -1 && (this.frames > this.maxFrames
        || (this.isIntaking && this.intakeSubsystem.getIntakeSwitch()));
    }
}
