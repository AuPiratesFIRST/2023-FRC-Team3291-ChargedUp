// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.RuntimeOperationsException;

import com.ctre.phoenix.platform.can.PlatformCAN;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;


public class DriveTrainSubsystems extends SubsystemBase {
  public final static int RIGHT = 1;
  public final static int LEFT = -1;

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

  public double DistanceInInches = 112;
  
  double breakAdj = 0.05;



    /*
   * Auto-balancing taken from: https://github.com/kauailabs/navxmxp/blob/master/roborio/java/navXMXP_Java_AutoBalance/src/org/usfirst/frc/team2465/robot/Robot.java
   */
  private AHRS navx_device;
  boolean autoBalanceXMode;
  boolean autoBalanceYMode; 
    
  /**
   * Runs the motors with arcade steering.
   */  
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;
  
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

  public SparkMaxPIDController pidController00;

  /** Creates a new DriveTrainSubsystems. */
  public DriveTrainSubsystems() {

    motorController01.follow(motorController00);
    motorController03.follow(motorController02);

    motorController00.getPIDController();
    motorController02.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double joystickLeftSpeed, double joystickRightSpeed) {
    double m_deadbad = Constants.DriveTrain.kDefaultDeadband;
    double m_maxOutput = Constants.DriveTrain.kDefaultMaxOutput;

    double leftSpeed = MathUtil.applyDeadband(joystickLeftSpeed, m_deadbad);
    double rightSpeed = MathUtil.applyDeadband(joystickRightSpeed, m_deadbad);

    leftSpeed = MathUtil.clamp(leftSpeed, -1.0, 1.0);
    rightSpeed = MathUtil.clamp(rightSpeed, -1.0, 1.0);

    motorController00.set(leftSpeed * m_maxOutput);
    motorController02.set(rightSpeed * m_maxOutput);
  }

  public void autoBalanceInitialize() {
    rotationsNeededLeft = encoder0.getPosition() + rotationsToBalance;
    rotationsNeededRight = encoder2.getPosition() + rotationsToBalance;
  }

  public void autoBalance() {
    double brakeAdjustment = Constants.DriveTrain.brakeAdjustment;

    double pitchAngleDegrees = navx_device.getPitch();
    double rollAngleDegrees = navx_device.getRoll();

    rollAngleDegrees = MathUtil.applyDeadband(rollAngleDegrees, 0.05);

    if (rollAngleDegrees != 0.0) {
      double radiansPerAngle = slope * rollAngleDegrees;

      double direction = 1.0;
      if (rollAngleDegrees < 0.0) {
        direction = -1.0;
      }

      radiansPerAngle = radiansPerAngle + (minMovementSpeed * direction);

      try {
        double encoder0Position = encoder0.getPosition();
        double encoder2Position = encoder2.getPosition();

        double leftSpeed;
        if (encoder0Position >= rotationsNeededLeft) {
          leftSpeed = radiansPerAngle - (brakeAdjustment * direction);
        } else {
          leftSpeed = radiansPerAngle;
        }

        double rightSpeed;
        if (encoder2Position >= rotationsNeededRight) {
          rightSpeed = radiansPerAngle - (brakeAdjustment * direction);
        } else {
          rightSpeed = radiansPerAngle;
        }

        drive(leftSpeed, rightSpeed);
      } catch(RuntimeException ex) {
        String err_string = "Drive system error: " + ex.getMessage();
        DriverStation.reportError(err_string, true);
      }

      Timer.delay(0.005);
    }
  }
}

  public void moveForwardOrBack(int distanceInInches, double speed){

    double motorController0Position = encoder0.getPosition();
    double motorController2Position = encoder2.getPosition();
    double movement = distanceInInches * rotationsPerInch;

    motorController0Position += movement;
    motorController2Position += movement;

    encoder0.setPosition(motorController0Position);
    encoder2.setPosition(motorController2Position);

    motorController00.set(speed);
    motorController02.set(speed);

    while(encoder0.getPosition() <= motorController0Position && encoder2.getPosition() <= motorController2Position){
      encoder0.setPosition(motorController0Position);
      encoder2.setPosition(motorController2Position);
    }

    motorController00.set(0.0);
    motorController02.set(0.0);
    
    /*encoderSetting = stopAndReset; 

    motorController00.setMode(encoderSetting);
    motorController02.setMode(encoderSetting);*/

  }


    private void strafeLeftOrRight (int distanceInInches, double speed) 
    int backLeftPosition = motorController00.getCurrentPosition();
    int frontRightPosition = motorController02.getCurrentPosition();
    
    double movement = distanceInInches 112 movementInInches;
    
    motorController00 += movement;
    backLeftPosition -= movement;
    motorController02 -= movement;
    frontRightPosition += movement;

    motorController00.setTargetPosition(frontleftPosition);
    motorController02.setTargetPosition(frontrightPosition);

    enconderSetting = runToPostion;

    motorController00.setMode(encoderSetting);
    motorController02.setMode(encoderSetting);
    
    motorController00.setPower(0.5);
    motorController02.setPower(0.5);
        
