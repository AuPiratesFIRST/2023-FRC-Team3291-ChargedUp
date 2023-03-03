// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerForwardCone extends CommandBase {
  public IndexerSubsystem indexerSubsystem;
  public boolean status;
  /** Creates a new IndexerForwardCone. */
  public IndexerForwardCone(IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
  indexerSubsystem = indexer;
  status = false;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerSubsystem.forwardCone();
    SmartDashboard.putBoolean("Indexer Backward", isFinished());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    status = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return status;
  }
}
