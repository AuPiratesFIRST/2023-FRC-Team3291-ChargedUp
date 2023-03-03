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
public class Auto04 extends SequentialCommandGroup {
  /** Creates a new Auto04. */
  public Auto04(DriveTrainSubsystems drivetrian,IndexerSubsystem indexersubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveForward(drivetrian, 112, .45).withTimeout(1),
      new IntakeForward(intakeSubsystem).withTimeout(1),
      new MoveBackward(drivetrian, 112, .5).withTimeout(1),
      new IndexerBackward(indexersubsystem, null).withTimeout(1),
      new MoveForward(drivetrian, 112, .5).withTimeout(1),
      new TurnRight(drivetrian, 90, .5).withTimeout(1),
      new MoveForward(drivetrian, 48, .5).withTimeout(1),
      new IntakeForward(intakeSubsystem).withTimeout(1),
      new MoveBackward(drivetrian, 48, .5).withTimeout(1),
      new TurnRight(drivetrian, 90, .5).withTimeout(1),
      new MoveBackward(drivetrian, 112, .45)
    );
  }
}
