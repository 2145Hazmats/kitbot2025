// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PublicKey;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax rightLeader;

  private final ADIS16470_IMU gyro;
 
  private final DifferentialDrive drive;
  private double distance;
  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
        // leftFollower = new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
       // rightFollower = new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
       

       

       gyro = new ADIS16470_IMU();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
        // leftFollower.setCANTimeout(250);
        // rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    config.encoder.positionConversionFactor(Constants.DriveConstants.MotorTick2Feets);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
      // config.follow(leftLeader);
      // leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      // config.follow(rightLeader);
      // rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getdistance(){
    return distance;
  }

  public void feedMotors() {
    drive.feed();
  }

  @Override
  public void periodic() {
    distance = -(leftLeader.getEncoder().getPosition() + rightLeader.getEncoder().getPosition()) / 2;
    SmartDashboard.putNumber("distanceOfMotors", distance);
    SmartDashboard.putNumber("getAngle", gyro.getAngle());
    SmartDashboard.putNumber("complementaryXAngle", gyro.getXComplementaryAngle());
    SmartDashboard.putNumber("complementaryYAngle", gyro.getYComplementaryAngle());
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(
      CANDriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
      // Speed is multiplied by 0.75 so its not as fast and dies!
        () -> drive.arcadeDrive(-xSpeed.getAsDouble()*0.6, -zRotation.getAsDouble()*0.6), driveSubsystem);

      
  }
  public Command faceAngle(double setpoint, double speed) {

    return Commands.run(()-> {
    
    if (gyro.getAngle() < setpoint - 8){
      leftLeader.set(speed);
      rightLeader.set(-speed);
    } 
    else if (gyro.getAngle() > setpoint + 8) {
      leftLeader.set(-speed);
      rightLeader.set(speed);
    }
    else  {
      leftLeader.set(0);
      rightLeader.set(0);
    
    }}
    , this);
  }

  public void driveforward(double speed) {
    leftLeader.set(-speed);
    rightLeader.set(-speed);
  }

}
  
  
