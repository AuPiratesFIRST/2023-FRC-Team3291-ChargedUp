// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystems;

public class AutobalanceCommandV2 extends CommandBase {
  public DriveTrainSubsystems driveTrainSubsystems;
  public boolean status;
  public double platformWidth = 48;
  public double robotLength = 35;
  public double cpr = 10.53;
  public double wheelDiameter = 8;

  public double distanceToEdge =  (platformWidth - (platformWidth - robotLength)/2);
  public double movementPerInch = cpr/(wheelDiameter * Math.PI);
  public double rotationsToBalance =  (distanceToEdge * movementPerInch);

  public double leftDifference;
  public double rightDifference;

  public double leftDifferenceInit;
  public double rightDifferenceInit;

  public double position1;
  public double position2;

  /** Creates a new AutobalanceCommand. */
  public AutobalanceCommandV2(DriveTrainSubsystems drivetrainsubsystems) {
    driveTrainSubsystems = drivetrainsubsystems;
    
    addRequirements(drivetrainsubsystems);
    // Use addRequirements() here to declare subsystem dedpendencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystems.autoBalanceInitialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    position1 = driveTrainSubsystems.encoder0.getPosition();
    position2 = driveTrainSubsystems.encoder2.getPosition();
    // Calculate left/right difference by getting current position - initial
    leftDifference = position1 - leftDifferenceInit;
    rightDifference = position2 - rightDifferenceInit;

    double rollAngleDegrees = driveTrainSubsystems.navx_device.getRoll();

    rollAngleDegrees = MathUtil.applyDeadband(rollAngleDegrees, 0.1);

    SmartDashboard.putNumber("rollAngleDegress", rollAngleDegrees);

    //gets the pid output of getting the roll and calculating the error of going to 0
    //Gotten from Carter from team 4009, Thank you.
    double pidOut = driveTrainSubsystems.pidDrive.calculate(rollAngleDegrees, 0);

    // inline clamp
    if (pidOut > 1) {pidOut = 1;}
    else if (pidOut < -1) {pidOut = -1;}

    //applies the calulated error to the motors so they can move
    driveTrainSubsystems.drive(-pidOut, -pidOut);

    if(Math.abs(pidOut) < 0.1){
      status = true;
    }

    if(Math.abs(leftDifference) >= rotationsToBalance && Math.abs(rightDifference) >=rotationsToBalance){
      // If reached destination set status = true
      status = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
