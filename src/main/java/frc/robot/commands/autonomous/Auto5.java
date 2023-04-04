// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.IndexerBackward;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto5 extends SequentialCommandGroup {
  /** Creates a new Auto5. */
  public Auto5(DriveTrainSubsystems driveTrainSubsystems, IndexerSubsystem indexersubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new IndexerBackward(indexersubsystem).withTimeout(4),
    new MoveForwardV2(driveTrainSubsystems, 200, 0.3)//.withTimeout(4)
    //new MoveBackward(driveTrainSubsystems, 112, 0.3)
    );
  }
}
