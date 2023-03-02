// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.indexer.IndexerBackward;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.IndexerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2 extends SequentialCommandGroup {
  /** Creates a new Auto02. */
  public Auto2(DriveTrainSubsystems driveTrain, IndexerSubsystem indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IndexerBackward(indexer,null).withTimeout(1),
      new MoveForward(driveTrain, 30, 0.3),
      new AutobalanceCommand(driveTrain)
      );
  }
}
