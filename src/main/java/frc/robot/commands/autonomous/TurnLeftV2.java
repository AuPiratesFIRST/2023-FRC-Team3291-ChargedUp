// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystems;

public class TurnLeftV2 extends CommandBase {
  public DriveTrainSubsystems driveTrainSubsystems;
  public double rotateAngle;
  public double speed;
  public boolean status;
  public double initialAngle;
  public double angleDifference;
  /** Creates a new TurnRight. */
  public TurnLeftV2(DriveTrainSubsystems drive, double rotationOfTheAngle, double motorSpeed) {
    driveTrainSubsystems = drive;
    speed = motorSpeed;
    rotateAngle = rotationOfTheAngle;
    status = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //driveTrainSubsystems.rotateLeftOrRight(rotateAngle, speed);
    //SmartDashboard.putBoolean("Turn Left", isFinished()); 
    initialAngle =  driveTrainSubsystems.navx_device.getAngle();
    angleDifference = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystems.drive(speed, speed);
    angleDifference = driveTrainSubsystems.navx_device.getAngle() - initialAngle;
    if(Math.abs(angleDifference) <= rotateAngle){
      // If reached destination set status = true
      status = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystems.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return status;
  }
}
