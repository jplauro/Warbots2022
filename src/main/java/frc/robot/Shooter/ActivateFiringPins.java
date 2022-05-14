package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake.IntakeSubsystem;

public class ActivateFiringPins extends CommandBase {
    private FiringPins firingPins;
    private IntakeSubsystem intakeSubsystem;
    private int frames;

    public ActivateFiringPins(FiringPins firingPins, IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        this.firingPins = firingPins;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Ball was shot");
        this.intakeSubsystem.enableInnerIntakeMotor();
    }

    @Override
    public void execute() {
        if (++this.frames == 15) {
            this.intakeSubsystem.disableInnerIntakeMotor();
            this.firingPins.extendFiringPinsSolenoid();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.firingPins.retractFiringPinsSolenoid();
    }

    @Override
    public boolean isFinished() {
        return frames >= 60;
    }
}
