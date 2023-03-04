// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystems;

public class MoveBackwardV2 extends CommandBase {
  public DriveTrainSubsystems driveTrainSubsystems;
  public double distanceInInches;
  public double speed;
  public boolean status;

  public double position1;
  public double position2;
  
  public double movement;

  public double leftDifference;
  public double rightDifference;

  public double leftDifferenceInit;
  public double rightDifferenceInit;

  /** Creates a new MoveForwardV2. */
  public MoveBackwardV2(DriveTrainSubsystems driveTrainSubsystem, double distance, double motorspeed) {
    // Use addRequirements() here to declare subsystem dependencies.  
    this.driveTrainSubsystems = driveTrainSubsystem;
    distanceInInches = distance;
    speed = motorspeed;
    status = false;
    addRequirements(driveTrainSubsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get current encoder positions from drivetrainsubsystem
    driveTrainSubsystems.encoder0.setPosition(0);
    driveTrainSubsystems.encoder2.setPosition(0);
    position1 = driveTrainSubsystems.encoder0.getPosition();
    position2 = driveTrainSubsystems.encoder2.getPosition();
    // Calculate movement
    movement = distanceInInches * driveTrainSubsystems.movementPerInch;
    // Calculate initial left/right differences
    leftDifferenceInit = position1;
    rightDifferenceInit = position2;
    // Start motor
    
    
    //driveTrainSubsystems.drive(speed, speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    position1 = driveTrainSubsystems.encoder0.getPosition();
    position2 = driveTrainSubsystems.encoder2.getPosition();
    // Calculate left/right difference by getting current position - initial
     leftDifference = position1 - leftDifferenceInit;
     rightDifference = position2 - rightDifferenceInit;

     SmartDashboard.putNumber("Right Difference", rightDifference);
     SmartDashboard.putNumber("Left Difference", leftDifference);
     SmartDashboard.putNumber("Speed", speed);
     SmartDashboard.putBoolean("Status", status);
     driveTrainSubsystems.drive(speed, speed);
    // If difference <= movement
    System.out.println("RUNNING"); 
    if(Math.abs(leftDifference) >= movement && Math.abs(rightDifference) >= movement){
      // If reached destination set status = true
      status = true;
      System.out.println("DONE");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop motor
    driveTrainSubsystems.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return status;
  }
}
