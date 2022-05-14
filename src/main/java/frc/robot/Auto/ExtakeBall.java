package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Intake.IntakeSubsystem;

public class ExtakeBall extends SequentialCommandGroup {
    private final double EXTAKE_SPEED = -1;

    public ExtakeBall(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        addCommands(
            new InstantCommand(() -> intakeSubsystem.setInnerIntakeMotor(this.EXTAKE_SPEED)),
            new WaitCommand(2),
            new InstantCommand(intakeSubsystem::disableInnerIntakeMotor)
        );
    }
}
