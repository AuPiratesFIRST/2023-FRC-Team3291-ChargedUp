// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto01 extends SequentialCommandGroup {
  /** Creates a new Auto01. */
  public Auto01(DriveTrainSubsystems driveTrainSubsystems) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       new MoveForward(driveTrainSubsystems, 56.55, 0.1).withTimeout(1),
       new TurnLeft(driveTrainSubsystems, 45, 0.1).withTimeout(1),
       new MoveBackward(driveTrainSubsystems, 56.55, 0.1).withTimeout(1),
       new TurnRight(driveTrainSubsystems, 45, 0.1).withTimeout(1)
    );
  }
}
