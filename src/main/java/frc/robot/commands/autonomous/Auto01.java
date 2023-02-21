// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.IndexerFoward;
import frc.robot.commands.intake.IntakeForward;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto01 extends SequentialCommandGroup {
  /** Creates a new Auto01. */
  public Auto01(DriveTrainSubsystems driveTrainSubsystems, IndexerSubsystem indexersubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // make sure the robot gets put 36.19 inches away from the right wall
       new MoveForward(driveTrainSubsystems, 112, 0.3).withTimeout(1),
       new IntakeForward(intakeSubsystem),
       new MoveBackward(driveTrainSubsystems, 112, 0.3).withTimeout(1),
       new IndexerFoward(indexersubsystem).withTimeout(1),
       new TurnLeft(driveTrainSubsystems, 65, 0.3).withTimeout(1),
       new MoveForward(driveTrainSubsystems, 20, .35).withTimeout(1),
       new TurnRight(driveTrainSubsystems, 90, .3),
       new MoveForward(driveTrainSubsystems, 14, .35)
    );
  }
}
