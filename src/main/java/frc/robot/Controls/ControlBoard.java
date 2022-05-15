package frc.robot.Controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControlBoard {
    // Driver Controls
    public static JoystickButton
        winchHoldButton,
        intakeButton,
        extakeButton;

    public static DoubleSupplier
        backwardTrigger,
        forwardTrigger,
        rotationJoystick;

    // Operator Controls
    public static JoystickButton 
        extendArmsButton,
        raiseArmsButton,
        climbSequenceButton,
        toggleHooksButton,
        tankDriveAimButton,
        lowPoweredShotButton,
        toggleGyroButton,
        reverseShooterWheelsButton;

    public static TriggerPressed 
        aimTurretTrigger,
        fireTurretTrigger;
    
    public static DoubleSupplier
        manualAimingJoystick;

    private static XboxController driver, operator;

    public static void init() {
        driver = new XboxController(ControlConstants.DRIVER_CONTROLLER_ID);
        operator = new XboxController(ControlConstants.OPERATOR_CONTROLLER_ID);

        ControlBoard.initDriverControls();
        ControlBoard.initOperatorControls();
    }

    private static void initDriverControls() {
        intakeButton = new JoystickButton(driver, Button.kB.value);
        extakeButton = new JoystickButton(driver, Button.kA.value);
        winchHoldButton = new JoystickButton(driver, Button.kX.value);

        backwardTrigger = driver::getLeftTriggerAxis;
        forwardTrigger = driver::getRightTriggerAxis;
        rotationJoystick = driver::getLeftX;
    }

    private static void initOperatorControls() {
        extendArmsButton = new JoystickButton(operator, Button.kX.value);
        raiseArmsButton = new JoystickButton(operator, Button.kY.value);
        reverseShooterWheelsButton = new JoystickButton(operator, Button.kA.value);
        climbSequenceButton = new JoystickButton(operator, Button.kB.value);

        manualAimingJoystick = operator::getLeftX;
        aimTurretTrigger = new TriggerPressed(operator, Axis.kLeftTrigger.value);
        tankDriveAimButton = new JoystickButton(operator, Button.kLeftBumper.value);
        fireTurretTrigger = new TriggerPressed(operator, Axis.kRightTrigger.value);
        lowPoweredShotButton = new JoystickButton(operator, Button.kRightBumper.value);

        toggleHooksButton = new JoystickButton(operator, Button.kStart.value);
        toggleGyroButton = new JoystickButton(operator, Button.kBack.value); 
    }

    private static void setControllerRumble(XboxController controller, 
    boolean state, double frequency, RumbleType rumbleType) {
        double rumble = state ? frequency : 0;
        controller.setRumble(rumbleType, rumble);
    }

    public static void setDriverLowFreqRumble(boolean state) {
        setControllerRumble(driver, state, ControlConstants.DRIVER_RUMBLE_LOW, RumbleType.kLeftRumble);
    }

    public static void setDriverHighFreqRumble(boolean state) {
        setControllerRumble(driver, state, ControlConstants.DRIVER_RUMBLE_HIGH, RumbleType.kRightRumble);
    }

    public static void setDriverRumble(boolean state) {
        setDriverLowFreqRumble(state);
        setDriverHighFreqRumble(state);
    }

    public static void setOperatorLowFreqRumble(boolean state) {
        setControllerRumble(driver, state, ControlConstants.OPERATOR_RUMBLE_LOW, RumbleType.kLeftRumble);
    }

    public static void setOperatorHighFreqRumble(boolean state) {
        setControllerRumble(driver, state, ControlConstants.OPERATOR_RUMBLE_HIGH, RumbleType.kRightRumble);
    }

    public static void setOperatorRumble(boolean state) {
        setOperatorLowFreqRumble(state);
        setOperatorHighFreqRumble(state);
    }
}
