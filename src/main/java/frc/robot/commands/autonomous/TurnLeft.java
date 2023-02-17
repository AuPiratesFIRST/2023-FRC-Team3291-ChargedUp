// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystems;

public class TurnLeft extends CommandBase {
  public DriveTrainSubsystems driveTrainSubsystems;
  public double rotateAngle;
  public double speed;
  /** Creates a new TurnRight. */
  public TurnLeft(DriveTrainSubsystems drive, double motorSpeed, double rotationOfTheAngle) {
    driveTrainSubsystems = drive;
    speed = motorSpeed;
    rotateAngle = rotationOfTheAngle;
    motorSpeed = motorSpeed * -1;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystems.rotateLeftOrRight(rotateAngle, speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
