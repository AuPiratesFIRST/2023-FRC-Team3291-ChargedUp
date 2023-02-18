// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystems;

public class MoveForward extends CommandBase {
  public DriveTrainSubsystems driveTrainSubsystems;
  public double distanceInInches;
  public double speed;
  /** Creates a new MoveForward. */
  public MoveForward( DriveTrainSubsystems driveTrainSubsystem, double distance, double motorspeed) {
    // Use addRequirements() here to declare subsystem dependencies.
  driveTrainSubsystems = driveTrainSubsystem;
  distanceInInches = distance;
  speed = motorspeed * -1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   driveTrainSubsystems.moveForwardOrBack(distanceInInches, speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   driveTrainSubsystems.moveForwardOrBack(0, Constants.STOPPOWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
