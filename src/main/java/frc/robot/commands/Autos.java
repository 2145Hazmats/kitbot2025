// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;
//import frc.robot.commands.DriveForCheese;
public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem,CANRollerSubsystem roll) {
    return new DriveForCheese(driveSubsystem, 10).withTimeout(1).andThen(driveSubsystem.faceAngle(450, .3).withTimeout(9)).andThen(roll.runRoller(-.66));//.andThen(new DriveForCheese(driveSubsystem,6)); //Commands.run() {DriveForCheese(driveSubsystem);}
   
  } 
}
