package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Auto.ExtakeBall;
import frc.robot.Auto.OneBall;
import frc.robot.Auto.Taxi;
import frc.robot.Auto.TwoBalls;
import frc.robot.Climber.ClimberSubsystem;
import frc.robot.Climber.ControlPistons;
import frc.robot.Climber.ControlPistons.PistonMotion;
import frc.robot.Climber.ExtendArmsAndStow;
import frc.robot.Climber.RaiseAndGrab;
import frc.robot.Climber.WinchHold;
import frc.robot.Climber.WinchSubsystem;
import frc.robot.Controls.ControlBoard;
import frc.robot.Drive.DriveWithJoystick;
import frc.robot.Drive.Drivetrain;
import frc.robot.Intake.ControlIntake;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.SmartIntake;
import frc.robot.Intake.ControlIntake.IntakeMotion;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ManualAimingPID;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.TankDriveAiming;
import frc.robot.Shooter.TurretAimingPID;
import frc.robot.Shooter.ZeroTurnTable;
import frc.robot.Util.Limelight;
import frc.robot.Util.LEDs.LEDIdleCommand;
import frc.robot.Util.LEDs.LEDSubsystem;

public class RobotContainer {
    private Drivetrain drivetrain;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private FiringPins firingPins;
    private LazySusanSubsystem lazySusanSubsystem;
    private ClimberSubsystem climberSubsystem;
    private WinchSubsystem winchSubsystem;
    private LEDSubsystem ledSubsystem;

    private DriveWithJoystick driveWithJoystick;
    private SendableChooser<CommandBase> autoSelector = new SendableChooser<CommandBase>();

    private Field2d robotFieldWidget = new Field2d(); // TODO: include Robot odometry

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
        this.lazySusanSubsystem = new LazySusanSubsystem(this.drivetrain::getPose);
        this.climberSubsystem = new ClimberSubsystem();
        this.winchSubsystem = new WinchSubsystem();
        this.ledSubsystem = new LEDSubsystem();

        SmartDashboard.putData(new InstantCommand(this.lazySusanSubsystem::setHomePosition));
        SmartDashboard.putData(new ZeroTurnTable(this.lazySusanSubsystem));
    }

    private void initControls() {
        // operator
        ControlBoard.raiseArmsButton.whenPressed(
            new ControlPistons(this.getClimberSubsystem(), PistonMotion.RAISE)
        );

        ControlBoard.climbSequenceButton.whenPressed(new RaiseAndGrab(
            this.getWinchSubsystem(), this.getClimberSubsystem(), this.getIntakeSubsystem()
        ));

        ControlBoard.extendArmsButton.whenPressed(new ExtendArmsAndStow(
            this.getWinchSubsystem(), this.getClimberSubsystem(), this.getIntakeSubsystem()
        ));

        ControlBoard.toggleHooksButton.whenPressed(new InstantCommand(() ->
            this.getClimberSubsystem().setHangingSolenoid(
                !this.getClimberSubsystem().getHooksSolenoid()
            )
        ).withTimeout(0.3)); // Equivalent of 15 frames

        ControlBoard.winchHoldButton.whenPressed(new WinchHold(
            this.getWinchSubsystem(), 
            this.getWinchSubsystem().getWinchPosition(), Constants.holdTime
        ));

        // TODO: here, now make a unified aiming/flywheel spinup command that we can use
        // for both auto and tele

        ControlBoard.aimTurretTrigger.whileActiveOnce(new ParallelCommandGroup(
                new LimelightSpinUp(this.getShooterSubsystem()),
                new TurretAimingPID(this.getLazySusanSubsystem(), this.getRobotField(), this.getDrivetrain()::getPose)
        ));

        ControlBoard.tankDriveAimButton.whileActiveOnce(new ParallelCommandGroup(
                new LimelightSpinUp(this.getShooterSubsystem()),
                new TankDriveAiming(this.getDrivetrain())
        ));

        ControlBoard.toggleGyroButton.whenPressed(new InstantCommand(() -> {
            this.getLazySusanSubsystem().setIsGyroLocking(!this.getLazySusanSubsystem().getIsGyroLocking());
            this.getLazySusanSubsystem().setIsHubTracking(!this.getLazySusanSubsystem().getIsHubTracking());
        }));

        ControlBoard.fireTurretTrigger.whenActive(new ParallelCommandGroup(  
            new ActivateFiringPins(getFiringPins(), getIntakeSubsystem()),
            new InstantCommand(this::logShot)
        ));

        ControlBoard.reverseShooterWheelsButton.whenPressed(
            new InstantCommand(() -> this.getShooterSubsystem().setIsBackwards(true))
        );

        ControlBoard.reverseShooterWheelsButton.whenReleased(
            new InstantCommand(() -> this.getShooterSubsystem().setIsBackwards(false))
        );

        ControlBoard.lowShotButton.whileActiveOnce(
            new InstantCommand(() -> this.getShooterSubsystem().setTargetRPM(Constants.lowPoweredShotRPM))
            .andThen(new InstantCommand(this.getShooterSubsystem()::stopMotors))
        );

        ControlBoard.intakeButton.whileActiveOnce(new SmartIntake(this.getIntakeSubsystem()));
        ControlBoard.extakeButton.whileActiveOnce(new ControlIntake(this.getIntakeSubsystem(), IntakeMotion.EXTAKE));
    }

    public void initAuto() {
        this.autoSelector.setDefaultOption("Two-Ball", new TwoBalls(
            this, this.getDrivetrain(), this.getLazySusanSubsystem(), 
            this.getShooterSubsystem(), this.getFiringPins(), this.getIntakeSubsystem()
        ));
        
        this.autoSelector.addOption("One-Ball", new OneBall(
            this.getDrivetrain(), this.getLazySusanSubsystem(),
            this.getShooterSubsystem(), this.getFiringPins(), this.getIntakeSubsystem()
        ));

        this.autoSelector.addOption("Taxi", new Taxi(
            this.getDrivetrain(), this.getIntakeSubsystem(), this.getLazySusanSubsystem()
        ));

        this.autoSelector.addOption("Extake-Ball", new ExtakeBall(this.getIntakeSubsystem()));

        SmartDashboard.putData(this.autoSelector);
    }

    public void init() {
        this.lazySusanSubsystem.setDefaultCommand(new ManualAimingPID(this.lazySusanSubsystem, ControlBoard.getOperatorController()));
        this.ledSubsystem.setDefaultCommand(new LEDIdleCommand(this.ledSubsystem, this.intakeSubsystem, this.firingPins, this.lazySusanSubsystem));
        SmartDashboard.putData(robotFieldWidget);
        robotFieldWidget.getObject("Turret").setPose(new Pose2d());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public void logShot() {
        DataLogManager.log(
            "LeftRPM: " + this.getShooterSubsystem().getLeftRPM() 
            + " RightRPM: " + this.getShooterSubsystem().getRightRPM() 
            + " RPMSetpoint: " + this.getShooterSubsystem().getSetpoint() 
            + " AtSetpoint: " + this.getShooterSubsystem().atTargetRPM() 
            + " X LimeLight: " + Limelight.getTX() 
            + " Y LimeLight: " + Limelight.getTY() 
            + " EventName: " + DriverStation.getEventName() 
            + " MatchNumber: " + DriverStation.getMatchNumber() 
            + " MatchTime: " + DriverStation.getMatchTime()
        );
    }

    public ClimberSubsystem getClimberSubsystem() {
        return this.climberSubsystem;
    }

    public LazySusanSubsystem getLazySusanSubsystem() {
        return this.lazySusanSubsystem;
    }

    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return this.shooterSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return this.intakeSubsystem;
    }

    public FiringPins getFiringPins() {
        return this.firingPins;
    }

    public WinchSubsystem getWinchSubsystem() {
        return this.winchSubsystem;
    }

    public XboxController getOperatorController() {
        return ControlBoard.getOperatorController();
    }

    public void setTeleopDrive() {
        driveWithJoystick = new DriveWithJoystick(drivetrain, ControlBoard.getDriverController());
        drivetrain.setDefaultCommand(driveWithJoystick);
    }

    public void calibrateTurntable() {
        if (!this.getLazySusanSubsystem().getIsCal()) {
            new ZeroTurnTable(this.getLazySusanSubsystem()).schedule();
        }
    }

    public CommandBase getSelectedAuto() {
        return this.autoSelector.getSelected();
    }

    public Field2d getRobotField() {
        return this.robotFieldWidget;
    }
}
