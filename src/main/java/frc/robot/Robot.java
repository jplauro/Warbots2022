package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LedMode;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        this.robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        this.robotContainer.getDrivetrain().setBrake(true);
        this.robotContainer.getTurretSubsystem().setMotorMode(IdleMode.kCoast);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopInit() {
        if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }

        this.robotContainer.calibrateTurret();
        this.robotContainer.getIntakeSubsystem().disableInnerIntakeMotor();
        this.robotContainer.getShooterSubsystem().setOffsetSpeed(0);
        this.robotContainer.getTurretSubsystem().setMotorMode(IdleMode.kBrake);
        this.robotContainer.initTeleopCommands();
        this.robotContainer.getDrivetrain().setBrake(true);
        this.robotContainer.getClimberSubsystem().setWinchPosition(0);
    }

    @Override
    public void teleopExit() {
        Limelight.setLedMode(LedMode.OFF);
    }

    @Override
    public void autonomousInit() {
        Limelight.setLedMode(LedMode.ON);
        this.robotContainer.getShooterSubsystem().setOffsetSpeed(0);
        this.robotContainer.getTurretSubsystem().setMotorMode(IdleMode.kBrake);

        this.autonomousCommand = this.robotContainer.getSelectedAuto();

        if (this.autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        Limelight.setLedMode(LedMode.OFF);
    }

    @Override
    public void testInit() {
        LiveWindow.setEnabled(false); // This is needed to allow commands to run
        this.robotContainer.getTurretSubsystem().setMotorMode(IdleMode.kBrake);
        this.robotContainer.initTestCommands();
    }

    /**
     * Simulation Code
     */
    @Override
    public void simulationPeriodic() {
        // Here we calculate the battery voltage based on drawn current.
        // As our robot draws more power from the battery its voltage drops.
        // The estimated voltage is highly dependent on the battery's internal resistance.
        double drawCurrent = this.robotContainer.getDrivetrain().getDrawnCurrentAmps();
        // Current seems to be too high look into later
        drawCurrent += this.robotContainer.getShooterSubsystem().getDrawnCurrentAmps();
        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(13, 0.02, drawCurrent);

        SmartDashboard.putNumber("Robot/Total Current", drawCurrent);
        SmartDashboard.putNumber("Robot/Robot Volts", loadedVoltage);
        RoboRioSim.setVInVoltage(loadedVoltage);

        Pose2d robotPos = this.robotContainer.getDrivetrain().getPose();
        this.robotContainer.getRobotField().setRobotPose(robotPos);
        Pose2d hubPos = new Pose2d(7.940, 4.08, new Rotation2d()); // Position of the hub
        this.robotContainer.getRobotField().getObject("hub").setPose(hubPos);
        Pose2d turretPos = new Pose2d(robotPos.getTranslation(),
                robotPos.getRotation().plus(this.robotContainer.getTurretSubsystem().getRotation()));
        // Turret position
        this.robotContainer.getRobotField().getObject("Turret").setPose(turretPos);

        this.robotContainer.getShooterSubsystem().posSim.setPosition(turretPos);
        this.robotContainer.getShooterSubsystem().posSim.update(Constants.kSimUpdateTime);
    }
}
