// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeBackward;
import frc.robot.commands.intake.IntakeForward;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto03 extends SequentialCommandGroup {
  /** Creates a new Auto03. */
  public Auto03(DriveTrainSubsystems drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveForward(drivetrain, 112, 0.3).withTimeout(1),
      new IntakeForward(intake).withTimeout(1),
      new MoveBackward(drivetrain, 112, 0.3).withTimeout(1),
      new IntakeBackward(intake).withTimeout(1),
      new TurnRight(drivetrain, 0.3, 65).withTimeout(1)   
    );
  }
}
