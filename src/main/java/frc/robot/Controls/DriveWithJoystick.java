package frc.robot.Controls;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive.Drivetrain;

public class DriveWithJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private double modSpeed = ControlConstants.SPEED_HIGH;
    private double modRotation = ControlConstants.ROTATION_HIGH;
    private double rampRate = ControlConstants.RAMP_RATE;
    private int power = ControlConstants.SQUARE_INPUTS ? 2 : 1;
    private boolean turnInPlace = ControlConstants.TURN_IN_PLACE;

    public DriveWithJoystick(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        this.modSpeed = SmartDashboard.getNumber("Controls/Mod Speed", this.modSpeed);
        this.modRotation = SmartDashboard.getNumber("Controls/Mod Rotation", this.modRotation);
        this.rampRate = SmartDashboard.getNumber("Controls/Ramp Rate", this.rampRate);
        this.power = SmartDashboard.getBoolean("Controls/Square Inputs", this.power == 2) ? 2 : 1;
        this.turnInPlace = SmartDashboard.getBoolean("Controls/Turn In Place", this.turnInPlace);

        double leftTriggerInput = Math.pow(ControlBoard.backwardTrigger.getAsDouble(), this.power);
        double rightTriggerInput = Math.pow(ControlBoard.forwardTrigger.getAsDouble(), this.power);
        double rotationInput = Math.pow(ControlBoard.rotationJoystick.getAsDouble(), this.power);

        // Only apply the original sign if the power is even
        if (this.power % 2 == 0) {
            rotationInput *= Math.signum(ControlBoard.rotationJoystick.getAsDouble());
        }

        double direction = rightTriggerInput > leftTriggerInput ? rightTriggerInput : -leftTriggerInput;
        double speed = direction * this.modSpeed;
        double rotation = -rotationInput * this.modRotation;

        this.drivetrain.curvatureInput(speed, rotation, this.turnInPlace);
    }
}
