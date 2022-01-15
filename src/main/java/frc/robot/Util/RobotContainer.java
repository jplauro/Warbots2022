// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Drive.Drivetrain;
import frc.robot.Drive.RotateMotor;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drive.DriveWithJoystick;
import edu.wpi.first.wpilibj2.command.Command;
/** Add your docs here. */
public class RobotContainer {
    protected Drivetrain drivetrain = new Drivetrain();
    protected XboxController driver = new XboxController(0);
    

    protected DriveWithJoystick driveWithJoystick;
    protected RotateMotor rotateMotor;

    //public Command getDriveWithJoystick() {
        //return new DriveWithJoystick(drivetrain, driver);
    //}

    public void init() {
        driveWithJoystick = new DriveWithJoystick(drivetrain, driver);
        drivetrain.setDefaultCommand(driveWithJoystick);
        rotateMotor = new RotateMotor(drivetrain, driver);
    } 

}