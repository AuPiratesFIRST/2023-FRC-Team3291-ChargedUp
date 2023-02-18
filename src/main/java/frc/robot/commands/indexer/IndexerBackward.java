// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerBackward extends CommandBase {
    public IndexerSubsystem indexersubsystem;
  /** Creates a new IndexerBackward. */
 public IndexerBackward(IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
  indexersubsystem = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexersubsystem.backward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexersubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
