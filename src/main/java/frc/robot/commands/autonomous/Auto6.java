// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.IndexerBackward;
import frc.robot.commands.intake.IntakeForward;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto6 extends SequentialCommandGroup {
  /** Creates a new Auto6. */
  public Auto6(DriveTrainSubsystems driveTrainSubsystems, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IndexerBackward(indexerSubsystem).withTimeout(4)
      /*new MoveForward(driveTrainSubsystems, 112, 0.3).withTimeout(1),
      new IntakeForward(intakeSubsystem).withTimeout(0.5),
      new MoveBackward(driveTrainSubsystems, 112, 0.5).withTimeout(0.5),
      new IndexerBackward(indexerSubsystem,null).withTimeout(0.5),
      new MoveForward(driveTrainSubsystems, 112, 0.3).withTimeout(1),
      new TurnLeft(driveTrainSubsystems, 90, 0.3).withTimeout(1),
      new MoveForward(driveTrainSubsystems, 48, 0.2).withTimeout(1),
      new IntakeForward(intakeSubsystem).withTimeout(1),
      new MoveBackward(driveTrainSubsystems, 48, 0.25).withTimeout(1),
      new TurnRight(driveTrainSubsystems, 90, .25).withTimeout(1),
      new MoveBackward(driveTrainSubsystems, 112, 0.3).withTimeout(1),
      new IndexerBackward(indexerSubsystem, null)*/
    );
  }
}
