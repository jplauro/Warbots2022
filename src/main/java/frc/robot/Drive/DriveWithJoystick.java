package frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls.ControlConstants;

public class DriveWithJoystick extends CommandBase {
    private Drivetrain drivetrain;
    private XboxController driverXbox;
    private double speedConstant = DriveConstants.SPEED_HIGH;
    private double rotationConstant = DriveConstants.ROTATION_HIGH;
    private double openLoopRampRateConstant = DriveConstants.RAMP_RATE;

    public DriveWithJoystick(Drivetrain drivetrain, XboxController driverXbox) {
        this.drivetrain = drivetrain;
        this.driverXbox = driverXbox;
        addRequirements(this.drivetrain);

        SmartDashboard.putNumber("DriveWithJoystick/speed", this.speedConstant);
        SmartDashboard.putNumber("DriveWithJoystick/rotation", this.rotationConstant);
        SmartDashboard.putNumber("DriveWithJoystick/openloopramprate", this.openLoopRampRateConstant);
    }

    @Override
    public void execute() {
        // Xbox Controller Input
        int power = ControlConstants.SQUARE_INPUTS ? 2 : 1;
        double rightTriggerInput = Math.pow(this.driverXbox.getRightTriggerAxis(), power); // Forwards
        double leftTriggerInput = Math.pow(this.driverXbox.getLeftTriggerAxis(), power); // Backwards
        double rotationInput = Math.pow(this.driverXbox.getLeftX(), power);

        // Only apply the original sign if the power is even
        if (power % 2 == 0)
            rotationInput *= Math.signum(this.driverXbox.getLeftX());

        double direction = rightTriggerInput > leftTriggerInput ? rightTriggerInput : -leftTriggerInput;
        double speed = direction * this.speedConstant;
        double rotation = rotationInput * this.rotationConstant;

        this.drivetrain.curvatureInput(speed, rotation, ControlConstants.TURN_IN_PLACE);
    }
}
