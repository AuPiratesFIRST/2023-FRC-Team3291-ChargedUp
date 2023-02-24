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
import frc.robot.commands.AutobalanceCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto6 extends SequentialCommandGroup {
  /** Creates a new Auto6. */
  public Auto6(DriveTrainSubsystems driveTrainSubsystems, IndexerSubsystem indexersubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeForward(intakeSubsystem).withTimeout(0.5),
      new IndexerBackward(indexersubsystem,null).withTimeout(0.5),
      new MoveForward(driveTrainSubsystems, 112, 0.3).withTimeout(1),
      new IntakeForward(intakeSubsystem).withTimeout(0.5),
      new MoveBackward(driveTrainSubsystems, 112, 0.5).withTimeout(0.5),
      new IndexerBackward(indexersubsystem,null).withTimeout(0.5),
      new MoveForward(driveTrainSubsystems, 112, 0.3).withTimeout(1),
      new TurnLeft(driveTrainSubsystems, 90, 0.3).withTimeout(1),
      new MoveForward(driveTrainSubsystems, 36, 0.2).withTimeout(1),
      new TurnRight(driveTrainSubsystems, 90, 0.3).withTimeout(1),
      new AutobalanceCommand(driveTrainSubsystems)
    );
  }
}
