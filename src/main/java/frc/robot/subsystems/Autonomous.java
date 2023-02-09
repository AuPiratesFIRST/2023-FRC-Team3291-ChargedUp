// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Autonomous extends SubsystemBase {

  public CANSparkMax motorController00 = new CANSparkMax(
    Constants.DriveTrain.canMotorDeviceId01,
    MotorType.kBrushless
  );

  public CANSparkMax motorController01 = new CANSparkMax(
    Constants.DriveTrain.canMotorDeviceId02,
    MotorType.kBrushless
  );

  public CANSparkMax motorController02 = new CANSparkMax(
  Constants.DriveTrain.canMotorDeviceId03,
  MotorType.kBrushless 
  );


  public CANSparkMax motorController03 = new CANSparkMax(
  Constants.DriveTrain.canMotorDeviceId04,
  MotorType.kBrushless
  ); 
  
  public RelativeEncoder encoder0 = motorController00.getEncoder();
  public RelativeEncoder encoder2 = motorController02.getEncoder();


  /** Creates a new Autonomous. */
  public Autonomous() {

    motorController01.follow(motorController00);
    motorController03.follow(motorController02);

    encoder0.setPosition(0);
    encoder2.setPosition(0)

    motorController00

  }

  public void moveForwardOrBack(int distanceInInches, double speed){

    int frontleftPosition = motorController00.getCurrentPosition();
    int backLeftPosition = motorController00.getCurrentPosition();
    int frontRightPosition = motorController02.getCurrentPosition();
    int backRightPosition = motorController02.getCurrentPosition();
    double movement = distanceInInches * movementInInches;

    frontleftPosition += movement;
    backLeftPosition += movement;
    frontRightPosition += movement;
    backRightPosition += movement;

    motorController00.setTargetPosition(frontleftPosition);
    motorController00.setTargetPosition(backLeftPosition);
    motorController02.setTargetPosition(frontRightPosition);
    motorController02.setTargetPosition(backRightPosition);

    encoderSetting = runToPosition;


  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
