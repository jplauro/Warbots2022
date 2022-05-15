package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Auto.ExtakeBall;
import frc.robot.Auto.OneBall;
import frc.robot.Auto.Taxi;
import frc.robot.Auto.TwoBalls;
import frc.robot.Climber.ClimberConstants;
import frc.robot.Climber.ClimberSubsystem;
import frc.robot.Climber.ControlPistons;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ExtendArms;
import frc.robot.Climber.PistonSubsystem;
import frc.robot.Climber.RaiseAndGrab;
import frc.robot.Climber.WinchHold;
import frc.robot.Controls.ControlBoard;
import frc.robot.Controls.DriveWithJoystick;
import frc.robot.Drive.Drivetrain;
import frc.robot.Intake.ControlIntake;
import frc.robot.Intake.ControlIntake.IntakeMotion;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.SmartIntake;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterConstants;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.TestSetpointSpinUp;
import frc.robot.Turret.CalibrateTurret;
import frc.robot.Turret.ManualAimingPID;
import frc.robot.Turret.MoveTurretToPos;
import frc.robot.Turret.TankDriveAiming;
import frc.robot.Turret.TurretAimingPID;
import frc.robot.Turret.TurretSubsystem;
import frc.robot.Util.Limelight;
import frc.robot.Util.LEDs.LEDIdleCommand;
import frc.robot.Util.LEDs.LEDSubsystem;

public class RobotContainer {
    private Drivetrain drivetrain;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private FiringPins firingPins;
    private TurretSubsystem turretSubsystem;
    private ClimberSubsystem climberSubsystem;
    private PistonSubsystem pistonSubsystem;
    private LEDSubsystem ledSubsystem;

    private SendableChooser<CommandBase> autoSelector = new SendableChooser<>();
    private Field2d robotField = new Field2d(); // TODO: include Robot odometry

    public RobotContainer() {
		ControlBoard.init();
        this.initSubsystems();
        this.initControls();
        Limelight.init();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        this.init();
        this.initAuto();
    }

    private void initSubsystems() {
        this.drivetrain = new Drivetrain();
        this.intakeSubsystem = new IntakeSubsystem();
        this.shooterSubsystem = new ShooterSubsystem();
        this.firingPins = new FiringPins();
        this.turretSubsystem = new TurretSubsystem(this.getDrivetrain()::getPose);
        this.climberSubsystem = new ClimberSubsystem();
        this.pistonSubsystem = new PistonSubsystem();
        this.ledSubsystem = new LEDSubsystem();
    }

    private void initControls() {
        // Driver Controls
        ControlBoard.winchHoldButton.whenPressed(new WinchHold(
            this.getClimberSubsystem(), this.getClimberSubsystem().getWinchPosition(), 
            ClimberConstants.WINCH_HOLD_FRAMES));

        ControlBoard.intakeButton.whileActiveOnce(new SmartIntake(this.getIntakeSubsystem()));

        ControlBoard.extakeButton.whileActiveOnce(new ControlIntake(
            this.getIntakeSubsystem(), IntakeMotion.EXTAKE));

        // Operator Controls
        ControlBoard.extendArmsButton.whenPressed(new ExtendArms(
            this.getClimberSubsystem(), this.getPistonSubsystem(), this.getIntakeSubsystem()));

        ControlBoard.raiseArmsButton.whenPressed(new ControlPistons(
            this.getPistonSubsystem(), PistonMotion.RAISE));

        ControlBoard.climbSequenceButton.whenPressed(new RaiseAndGrab(
            this.getClimberSubsystem(), this.getPistonSubsystem(), this.getIntakeSubsystem()));

        ControlBoard.toggleHooksButton.whenPressed(new InstantCommand(
            () -> this.getPistonSubsystem().setHooksSolenoid(
                !this.getPistonSubsystem().getHooksSolenoid())));

        ControlBoard.tankDriveAimButton.whileActiveOnce(new TankDriveAiming(
            this.getDrivetrain(), this.getTurretSubsystem())
            .alongWith(new LimelightSpinUp(this.getShooterSubsystem())));

        ControlBoard.lowPoweredShotButton.whileActiveOnce(new InstantCommand(
            () -> this.getShooterSubsystem().setTargetRPM(ShooterConstants.LOW_POWERED_SHOT_RPM)
        )).whenInactive(new InstantCommand(this.getShooterSubsystem()::stopMotors));

        ControlBoard.toggleGyroButton.whenPressed(new InstantCommand(() -> {
            this.getTurretSubsystem().setIsGyroLocking(!this.getTurretSubsystem().getIsGyroLocking());
            this.getTurretSubsystem().setIsHubTracking(!this.getTurretSubsystem().getIsHubTracking());
        }));

        ControlBoard.reverseShooterWheelsButton.whenPressed(new InstantCommand(
            () -> this.getShooterSubsystem().setIsBackward(true)
        )).whenReleased(new InstantCommand(() -> this.getShooterSubsystem().setIsBackward(false)));

        ControlBoard.aimTurretTrigger.whileActiveOnce(new TurretAimingPID(
            this.getTurretSubsystem(), this.getRobotField(), this.getDrivetrain()::getPose)
            .alongWith(new LimelightSpinUp(this.getShooterSubsystem())));

        ControlBoard.fireTurretTrigger.whenActive( 
            new ActivateFiringPins(this.getFiringPins(), this.getIntakeSubsystem())
            .alongWith(new InstantCommand(this::logShot)));
    }

    public void initAuto() {
        this.autoSelector.setDefaultOption("Two-Ball", new TwoBalls(
            this, this.getDrivetrain(), this.getTurretSubsystem(), 
            this.getShooterSubsystem(), this.getFiringPins(), this.getIntakeSubsystem()));
        
        this.autoSelector.addOption("Extake-Ball", new ExtakeBall(this.getIntakeSubsystem()));

        this.autoSelector.addOption("One-Ball", new OneBall(
            this.getDrivetrain(), this.getTurretSubsystem(),
            this.getShooterSubsystem(), this.getFiringPins(), this.getIntakeSubsystem()));

        this.autoSelector.addOption("Taxi", new Taxi(this.getDrivetrain(), this.getTurretSubsystem()));

        SmartDashboard.putData(this.autoSelector);
    }

    public void init() {
        this.ledSubsystem.setDefaultCommand(new LEDIdleCommand(this.ledSubsystem, 
            this.intakeSubsystem, this.firingPins, this.turretSubsystem));
        this.robotField.getObject("Turret").setPose(new Pose2d());
        SmartDashboard.putData(this.robotField);
    }

    public void initTeleopCommands() {
        this.getDrivetrain().setDefaultCommand(new DriveWithJoystick(this.getDrivetrain()));
        this.getTurretSubsystem().setDefaultCommand(new ManualAimingPID(this.getTurretSubsystem()));
    }

    public void initTestCommands() {
        this.getTurretSubsystem().setDefaultCommand(new MoveTurretToPos(this.getTurretSubsystem()));
        new TestSetpointSpinUp(this.getShooterSubsystem()).schedule();
    }

    public void calibrateTurret() {
        if (!this.getTurretSubsystem().getIsCalibrated()) {
            new CalibrateTurret(this.getTurretSubsystem()).schedule();
        }
    }

    public void logShot() {
        DataLogManager.log(
            "LeftRPM: " + this.getShooterSubsystem().getLeftRPM() 
            + " RightRPM: " + this.getShooterSubsystem().getRightRPM() 
            + " RPMSetpoint: " + this.getShooterSubsystem().getTargetRPM() 
            + " AtSetpoint: " + this.getShooterSubsystem().atTargetRPM() 
            + " X LimeLight: " + Limelight.getTX() 
            + " Y LimeLight: " + Limelight.getTY() 
            + " EventName: " + DriverStation.getEventName() 
            + " MatchNumber: " + DriverStation.getMatchNumber() 
            + " MatchTime: " + DriverStation.getMatchTime()
        );
    }

    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return this.intakeSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return this.shooterSubsystem;
    }

    public FiringPins getFiringPins() {
        return this.firingPins;
    }

    public TurretSubsystem getTurretSubsystem() {
        return this.turretSubsystem;
    }

    public ClimberSubsystem getClimberSubsystem() {
        return this.climberSubsystem;
    }

    public PistonSubsystem getPistonSubsystem() {
        return this.pistonSubsystem;
    }

    public CommandBase getSelectedAuto() {
        return this.autoSelector.getSelected();
    }

    public Field2d getRobotField() {
        return this.robotField;
    }
}
