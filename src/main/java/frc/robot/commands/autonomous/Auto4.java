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
import frc.robot.commands.autonomous.MoveForwardV2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4 extends SequentialCommandGroup {
  /** Creates a new Auto04. */
  public Auto4(DriveTrainSubsystems drivetrian,IndexerSubsystem indexer, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveForwardV2(drivetrian, 90, 0.2)
      /*new IndexerBackward(indexer).withTimeout(1),
      new MoveForward(drivetrian, 112, 0.35),
      new IntakeForward(intakeSubsystem).withTimeout(1),
      new MoveBackward(drivetrian, 112, 0.35),
      new IndexerBackward(indexer).withTimeout(1),
      new MoveForward(drivetrian, 112, 0.35),
      new TurnRight(drivetrian, 90, .3),
      new MoveForward(drivetrian, 48, 0.35),
      new IntakeForward(intakeSubsystem).withTimeout(1),
      new MoveBackward(drivetrian, 48, 0.35),
      new TurnLeft(drivetrian, 90, 0.35)*/
    );
  }
}
