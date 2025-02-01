// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForCheese extends Command {
  
  /** Creates a new DriveForCheese. */
  private final CANDriveSubsystem m_driveSubsystem;
  private final double setpointcheeseyo = 10;
  public DriveForCheese(CANDriveSubsystem drive, double setpointcheese) {
    m_driveSubsystem = drive;
    addRequirements(m_driveSubsystem);
    double setpointcheeseyo = setpointcheese;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.resetMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if (m_driveSubsystem.getdistance() < 10) {
      m_driveSubsystem.driveforward(0.32);
    }
    else if (m_driveSubsystem.getdistance() >= 10){
      m_driveSubsystem.driveforward(0);
    }

    m_driveSubsystem.feedMotors();

    SmartDashboard.putNumber("setpointcheeseyo", setpointcheeseyo);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_driveSubsystem.getdistance() >= 10) {
      return true;
    }
    else {
      return false;
    }
   
  }
}
