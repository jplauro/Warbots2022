package frc.robot.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax innerIntakeMotor;
    private final CANSparkMax intakeArmsMotor;
    private final Solenoid intakeArmsSolenoidA;
    private final Solenoid intakeArmsSolenoidB;
    private final DigitalInput intakeLimitSwitch;
    private final Debouncer intakeLimitSwitchDebouncer;

    public IntakeSubsystem() {
        this.innerIntakeMotor = new CANSparkMax(IntakeConstants.INNER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.intakeArmsMotor = new CANSparkMax(IntakeConstants.INTAKE_ARMS_MOTOR_ID, MotorType.kBrushless);
        this.intakeArmsSolenoidA = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_ARMS_SOLENOID_A_ID);
        this.intakeArmsSolenoidB = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_ARMS_SOLENOID_B_ID);
        this.intakeLimitSwitch = new DigitalInput(IntakeConstants.INTAKE_LIMIT_SWITCH_ID);
        // Only debounce on the rising edge (false --> true)
        this.intakeLimitSwitchDebouncer = new Debouncer(IntakeConstants.INTAKE_LIMIT_SWITCH_DEBOUNCE_TIME);

        for (CANSparkMax motor : new CANSparkMax[] {this.innerIntakeMotor, this.intakeArmsMotor}) {
            motor.restoreFactoryDefaults();
            motor.setInverted(true);
            motor.setIdleMode(IntakeConstants.BRAKE_WHEN_IDLE ? IdleMode.kBrake : IdleMode.kCoast);
            motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        }

        this.intakeArmsSolenoidA.set(false);
        this.intakeArmsSolenoidB.set(false);
    }

    public void enableInnerIntakeMotor() {
        this.innerIntakeMotor.set(1);
    }

    public void reverseInnerIntakeMotor() {
        this.innerIntakeMotor.set(-1);
    }

    public void disableInnerIntakeMotor() {
        this.innerIntakeMotor.set(0);
    }

    public void setInnerIntakeMotor(double speed) {
        this.innerIntakeMotor.set(speed);
    }

    public void enableIntakeArmsMotor() {
        this.intakeArmsMotor.set(-0.6);
    }

    public void disableIntakeArmsMotor() {
        this.intakeArmsMotor.set(0);
    }

    public void reverseIntakeArmsMotor() {
        this.intakeArmsMotor.set(0.6);
    }

    public void setIntakeArmsMotor(double speed) {
        this.intakeArmsMotor.set(speed);
    }

    public void extendIntakeArms() {
        this.intakeArmsSolenoidA.set(true);
        this.intakeArmsSolenoidB.set(true);
    }
    
    public void retractIntakeArms() {
        this.intakeArmsSolenoidA.set(false);
        this.intakeArmsSolenoidB.set(false);    
    }

    public void floatIntakeArms() {
        this.intakeArmsSolenoidA.set(false);
        this.intakeArmsSolenoidB.set(true);    
    }

    public boolean getIntakeSwitch() {
        return this.intakeLimitSwitchDebouncer.calculate(this.intakeLimitSwitch.get());
    }
}
