package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class MoveTurretToPos extends CommandBase {
    private LazySusanSubsystem lazySusanSubsystem;
    private double x = Constants.stowedDegrees;

    public MoveTurretToPos(LazySusanSubsystem lazySusanSubsystem) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        addRequirements(this.lazySusanSubsystem);
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("TestPos: ", this.x);
    }

    @Override
    public void execute() {
        this.x = SmartDashboard.getNumber("TestPos: ", this.x);
        this.lazySusanSubsystem.setTurretPosition(Rotation2d.fromDegrees(this.x));   
    }
}
