// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForwardsCube extends CommandBase {
  public IntakeSubsystem intakesubsystem;
  public boolean status;
  /** Creates a new IntakeForwardsCone. */
  public IntakeForwardsCube(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    status = false;
    intakesubsystem = intake; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakesubsystem.forwardCube();
    SmartDashboard.putBoolean("Intake Forward Cone", isFinished()); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    status = true;
    intakesubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return status;
  }
}
