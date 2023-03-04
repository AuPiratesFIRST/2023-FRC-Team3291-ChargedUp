// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrainSubsystems extends SubsystemBase {
  public final static double RIGHT = 1;
  public final static double LEFT = -1;

  double cpr = 10.53;
  double robotRadius = 21.5;
  double wheelDiameter = 8;
  double platformWidth = 48;
  double robotLength = 28;
  double minAngle = 0;
  double minMovementSpeed = 0.1;
  double maxAngle = 15;
  double maxMovementSpeed = 0.4;

  public double kDefaultDeadband = 0.02;
  public double kDefaultMaxOutput = 1.0;

  public double movementPerInch = cpr/(wheelDiameter * Math.PI);
  public double distanceToEdge =  platformWidth - (platformWidth - robotLength)/2;
  public double rotationsToBalance =  (distanceToEdge * movementPerInch);
  public double slope = (maxMovementSpeed - minMovementSpeed) / (maxAngle - minAngle);
  public double robotPerDegree = (( robotRadius / (wheelDiameter / 2 )) * cpr) / 360;

  public double radiansPerAngle; 

  public double rotationsInitRight;
  public double rotationsInitLeft;

  public double invert = -1;

  public double DistanceInInches = 112;
  
  double breakAdj = 0.05;

  public PIDController pidDrive;



    /*
   * Auto-balancing taken from: https://github.com/kauailabs/navxmxp/blob/master/roborio/java/navXMXP_Java_AutoBalance/src/org/usfirst/frc/team2465/robot/Robot.java
   */
  public AHRS navx_device;
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
  public SparkMaxPIDController pidController01;


  /** Creates a new DriveTrainSubsystems. */
  public DriveTrainSubsystems() {
    navx_device = new AHRS(SerialPort.Port.kUSB);

    motorController01.follow(motorController00);
    motorController03.follow(motorController02);

    motorController00.setInverted(true);

    motorController00.getPIDController();
    motorController02.getPIDController();

    SmartDashboard.putNumber("movementPerInch", movementPerInch);
    SmartDashboard.putNumber("distanceToEdge", distanceToEdge);
    SmartDashboard.putNumber("rotationsToBalance", rotationsToBalance);
    SmartDashboard.putNumber("slope", slope);
    SmartDashboard.putNumber("robotPerDegre", robotPerDegree);
    //SmartDashboard.putNumber("speedModifier", 1.0);
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

    //double speedModifier = SmartDashboard.getNumber("speedModifier", 0.1);

    //leftSpeed = leftSpeed;//* speedModifier;
    //rightSpeed = rightSpeed;// * speedModifier;
    SmartDashboard.putNumber("Left Set", joystickLeftSpeed);
    SmartDashboard.putNumber("Right Set", rightSpeed * m_maxOutput);
     
   motorController00.set(leftSpeed * m_maxOutput);
   motorController02.set(rightSpeed * m_maxOutput);
  }

  public void autoBalanceInitialize() {
    navx_device.reset();
    //encoder0.setPosition(0.0);
    //encoder2.setPosition(0.0);
    rotationsInitLeft = encoder0.getPosition();
    rotationsInitRight = encoder2.getPosition();
    SmartDashboard.putNumber("Left init position", rotationsInitLeft);
    SmartDashboard.putNumber("Right init position", rotationsInitRight);
    pidDrive = new PIDController(Constants.DriveTrain.kPDrive, Constants.DriveTrain.kIDrive, Constants.DriveTrain.kDDrive);
  }

  public void autoBalance() {
    double brakeAdjustment = Constants.DriveTrain.brakeAdjustment;

    double rollAngleDegrees = navx_device.getRoll();

    rollAngleDegrees = MathUtil.applyDeadband(rollAngleDegrees, 0.1);

    SmartDashboard.putNumber("rollAngleDegress", rollAngleDegrees);

    double differenceLeft = encoder0.getPosition() - rotationsInitLeft;
    double differenceRight = encoder2.getPosition() - rotationsInitRight;

    SmartDashboard.putNumber("differenceLeft1", differenceLeft);
    SmartDashboard.putNumber("differenceRight1", differenceRight);

    //gets the pid output of getting the roll and calculating the error of going to 0
    //Gotten from Carter from team 4009, Thank you.
    double pidOut = pidDrive.calculate(rollAngleDegrees, 0);

    // inline clamp
    if (pidOut > 1) {pidOut = 1;}
    else if (pidOut < -1) {pidOut = -1;}

    //applies the calulated error to the motors so they can move
    drive(pidOut, -pidOut);
    
    /* 
    if (rollAngleDegrees != 0.0) {
       double radiansPerAngle = slope * rollAngleDegrees;

       SmartDashboard.putNumber("radiansPerAngle", radiansPerAngle);

       double direction = 1.0;
       if (rollAngleDegrees < 0.0) {
         direction = -1.0;
       }

       SmartDashboard.putNumber("direction", direction);

       radiansPerAngle = radiansPerAngle + (minMovementSpeed * direction);

       SmartDashboard.putNumber("calcRadiansPerAngle", radiansPerAngle);
       try {
         double encoder0Position = encoder0.getPosition();
         double encoder2Position = encoder2.getPosition();

         SmartDashboard.putNumber("enc0Pos", encoder0Position);
         SmartDashboard.putNumber("enc2Pos", encoder2Position);
         SmartDashboard.putNumber("rotationsInitLeft", rotationsInitLeft);
         SmartDashboard.putNumber("rotationinitRight", rotationsInitRight);
         SmartDashboard.putNumber("Difference left", differenceLeft);
         SmartDashboard.putNumber("Difference right", differenceRight);

         //differenceLeft = pidDrive.calculate(getDistance(), rotationsToBalance);
         //differenceRight = differenceLeft;

         SmartDashboard.putNumber("differenceLeft2", differenceLeft);
         SmartDashboard.putNumber("differenceRight2", differenceRight);

         double leftSpeed;
         if (Math.abs(differenceLeft) >= rotationsToBalance) {
           leftSpeed = radiansPerAngle - (brakeAdjustment * direction);
         } else {
           leftSpeed = radiansPerAngle;
         }

         double rightSpeed;
         if (Math.abs(differenceRight) >= rotationsToBalance) {
           rightSpeed = radiansPerAngle - (brakeAdjustment * direction);
         } else {
           rightSpeed = radiansPerAngle;
         }

         SmartDashboard.putNumber("rightSpeed1", rightSpeed);
         SmartDashboard.putNumber("leftSpeed1", leftSpeed);

         leftSpeed = MathUtil.clamp(leftSpeed, -maxMovementSpeed, maxMovementSpeed);
         rightSpeed = MathUtil.clamp(rightSpeed, -maxMovementSpeed, maxMovementSpeed);

         drive(leftSpeed, rightSpeed);

         SmartDashboard.putNumber("rightSpeed2", rightSpeed);
         SmartDashboard.putNumber("leftSpeed2", leftSpeed);

         SmartDashboard.putNumber("Encoder left position", encoder0.getPosition());
         SmartDashboard.putNumber("Encoder right position", encoder2.getPosition());
      
       } catch(RuntimeException ex) {
         String err_string = "Drive system error: " + ex.getMessage();
         DriverStation.reportError(err_string, true);
       }

    //   Timer.delay(0.005);
      }
      */
  } 

  public void moveForwardOrBack(double distanceInInches, double speed){

    //pidDrive = new PIDController(Constants.DriveTrain.kPDrive, Constants.DriveTrain.kIDrive, Constants.DriveTrain.kDDrive);

    double motorController0Position = encoder0.getPosition();
    double motorController2Position = encoder2.getPosition();
    double movement = distanceInInches * movementPerInch;

    //double movement = 50;

    SmartDashboard.putNumber("movement", movement);

    //double leftDifference = pidDrive.calculate(getDistance(), distanceInInches);
    //double rightDifference = leftDifference;

    double leftDifference = motorController0Position - motorController0Position;
    double rightDifference = motorController2Position - motorController2Position;

    SmartDashboard.putNumber("Motorcontroller 0 position", motorController0Position);
    SmartDashboard.putNumber("Motorcontroller 2 position", motorController2Position);

    motorController00.set(speed);
    motorController02.set(speed);


    while(Math.abs(leftDifference) <= movement && Math.abs(rightDifference) <= movement){
      leftDifference = encoder0.getPosition() - motorController0Position;
      rightDifference = encoder2.getPosition() - motorController2Position;
      SmartDashboard.putNumber("left Difference", leftDifference);
      SmartDashboard.putNumber("right Difference", rightDifference);
      
    }

    motorController00.set(0.0);
    motorController02.set(0.0);
  }


    public void rotateLeftOrRight (double rotateAngle, double speed) { 

      /*encoder0.setPosition(0);
      encoder2.setPosition(0);
      //pidDrive = new PIDController(Constants.DriveTrain.kPDrive, Constants.DriveTrain.kIDrive, Constants.DriveTrain.kDDrive);

      double motorController0Position = encoder0.getPosition();
      double motorController2Position = encoder2.getPosition();
      //double movement = rotateAngle * (robotPerDegree/2);
      double movement = (((rotateAngle -8.57143)/(15/6.69))*robotPerDegree);

      SmartDashboard.putNumber("Movement", movement);
      SmartDashboard.putNumber("Rotations of the angle", rotateAngle);
  
      double leftDifference = motorController0Position - motorController0Position;
      double rightDifference = motorController2Position - motorController2Position;*/

      //leftDifference = pidDrive.calculate(getDistance(), rotateAngle);
      //rightDifference = leftDifference;

      motorController00.set(speed);
      motorController02.set(-1 * speed);

      double initialAngle = navx_device.getAngle();
      double angleDifference = 0;
      
      while(Math.abs(angleDifference) <= rotateAngle){
        angleDifference = navx_device.getAngle() - initialAngle; // - motorController2Position;
        SmartDashboard.putNumber("angle Difference", angleDifference);
       
      }
  
      motorController00.set(0.0);
      motorController02.set(0.0);
    }

    public double getDistance(){

      //encoder0.setPosition(0);
      //encoder2.setPosition(0);

      return ((encoder0.getPosition() + encoder2.getPosition())/2);

    }

  }