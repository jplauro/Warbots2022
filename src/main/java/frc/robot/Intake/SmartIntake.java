package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private int frames = 0;

    public SmartIntake(IntakeSubsystem intakeSubsystem) {   
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.frames = 0;
        this.intakeSubsystem.enableIntakeArmsMotor();
        this.intakeSubsystem.enableInnerIntakeMotor();
        this.intakeSubsystem.extendIntakeArms();
    }

    @Override
    public void execute() {
        if (++this.frames > IntakeConstants.FRAMES_UNTIL_FLOAT) {
            intakeSubsystem.floatIntakeArms();
        }

        if (this.intakeSubsystem.getIntakeSwitch()) {
            this.intakeSubsystem.disableInnerIntakeMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.disableInnerIntakeMotor();
        this.intakeSubsystem.disableIntakeArmsMotor();
        this.intakeSubsystem.retractIntakeArms();
    }
}
