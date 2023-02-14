// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Autonomous extends SubsystemBase {

  double diameter = 8;
  double platformWidth = 48;
  double robotLength = 28;
  double minAngle = 0;
  double minMovementSpeed = 0.3;
  double maxAngle = 15;
  double maxMovementSpeed = 0.7;

  public double kDefaultDeadband = 0.02;
  public double kDefaultMaxOutput = 1.0;

  public double rotationsPerInch = 1/(diameter * Math.PI);
  public double distanceToEdge =  platformWidth - (platformWidth - robotLength)/2;
  public double rotationsToBalance =  distanceToEdge * rotationsPerInch;
  public double slope = (maxMovementSpeed - minMovementSpeed) / (maxAngle - minAngle);

  public double radiansPerAngle;

  public double rotationsNeededRight;
  public double rotationsNeededLeft;

  public double invert = -1;

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
    encoder2.setPosition(0);

    //motorController00

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
