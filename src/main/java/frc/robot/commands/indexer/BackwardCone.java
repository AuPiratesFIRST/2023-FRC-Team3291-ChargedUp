// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LightingSubsystem;

public class BackwardCone extends CommandBase {
    public IndexerSubsystem indexersubsystem;
    public LightingSubsystem lightingSubsystem;
    public boolean status;
  /** Creates a new IndexerBackward. */
 public BackwardCone(IndexerSubsystem indexer, LightingSubsystem light) {
    // Use addRequirements() here to declare subsystem dependencies.
  indexersubsystem = indexer;
  lightingSubsystem = light;
  status = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexersubsystem.backwardCone();
    SmartDashboard.putBoolean("Indexer Backward", isFinished()); 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lightingSubsystem.blink();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    status = true;
    indexersubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return status;
  }
}
