package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerPressed extends Trigger {
    private final XboxController controller;
    private final int axis;

    public TriggerPressed(XboxController controller, int axisNumber) {
        this.controller = controller;
        this.axis = axisNumber;
    }

    @Override
    public boolean get() {
        double axisValue = Math.abs(this.controller.getRawAxis(this.axis));
        return axisValue > ControlConstants.DEADZONE;
    }
}
