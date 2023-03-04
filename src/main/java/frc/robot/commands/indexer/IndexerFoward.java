// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerFoward extends CommandBase {
  public IndexerSubsystem indexersubsystem;
  /** Creates a new IndexerFoward. */
  public IndexerFoward(IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
  indexersubsystem = indexer;
  addRequirements(indexersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //indexersubsystem.forward();
    SmartDashboard.putBoolean("Indexer forward", isFinished()); 

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
