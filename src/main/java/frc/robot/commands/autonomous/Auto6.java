// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.commands.intake.IntakeBackward;
import frc.robot.commands.intake.IntakeForward;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Auto6 extends SequentialCommandGroup {
  /** Creates a new Auto6. */
  public Auto6(DriveTrainSubsystems drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake) {
  
    addCommands(
      new MoveForward(drivetrain, 112, 0.3).withTimeout(1),
      new IntakeForward(intake).withTimeout(1),
      new MoveBackward(drivetrain, 112, .3),
      new IndexerSubsystem()
   )
  }
}
