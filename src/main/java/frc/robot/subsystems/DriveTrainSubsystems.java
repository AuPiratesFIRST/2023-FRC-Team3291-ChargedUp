// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrainSubsystems extends SubsystemBase {
  public final static int RIGHT = 1;
  public final static int LEFT = -1;

  double diameter = 6;
  double platformWidth = 48;
  double robotLength = 28;
  double minAngle = 0;
  double minMovementSpeed = 0.1;
  double maxAngle = 15;
  double maxMovementSpeed = 0.4;

  public IdleMode encoderSetting;
  // private IdleMode stopAndReset = DCMotor.RunMode.STOP_AND_RESET_ENCODER;
  // private IdleMode runToPosition = .RunMode.RUN_TO_POSITION;
 // private IdleMode runUsingEncoder = .RunMode.RUN_USING_ENCODER;

  public double kDefaultDeadband = 0.02;
  public double kDefaultMaxOutput = 1.0;

  public double rotationsPerInch = 1/(diameter * Math.PI);
  public double distanceToEdge =  platformWidth - (platformWidth - robotLength)/2;
  public double rotationsToBalance =  (distanceToEdge * rotationsPerInch)*10;
  public double slope = (maxMovementSpeed - minMovementSpeed) / (maxAngle - minAngle);

  public double radiansPerAngle;

  public double rotationsInitRight;
  public double rotationsInitLeft;

  public double invert = -1;

  public double DistanceInInches = 112;
  
  double breakAdj = 0.05;



    /*
   * Auto-balancing taken from: https://github.com/kauailabs/navxmxp/blob/master/roborio/java/navXMXP_Java_AutoBalance/src/org/usfirst/frc/team2465/robot/Robot.java
   */
  private AHRS navx_device = new AHRS(SerialPort.Port.kUSB);
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

    motorController00.setInverted(true);

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

    SmartDashboard.putNumber("Drive encoder Left", encoder0.getPosition());
    SmartDashboard.putNumber("Drive encoder Right", encoder2.getPosition());
  }

  public void autoBalanceInitialize() {
    navx_device.reset();
    encoder0.setPosition(0.0);
    encoder2.setPosition(0.0);
    rotationsInitLeft = encoder0.getPosition();
    rotationsInitRight = encoder2.getPosition();
  }

  public void autoBalance() {
    double brakeAdjustment = Constants.DriveTrain.brakeAdjustment;

    //double pitchAngleDegrees = navx_device.getPitch();
    double rollAngleDegrees = navx_device.getRoll();
    //SmartDashboard.putNumber("Pitch angle degrees", pitchAngleDegrees);
    SmartDashboard.putNumber("Roll angle degrees", rollAngleDegrees);

    rollAngleDegrees = MathUtil.applyDeadband(rollAngleDegrees, 0.1);
    SmartDashboard.putNumber("rollAngleDegrees", rollAngleDegrees);

    double differenceLeft = encoder0.getPosition() - rotationsInitLeft;
    double differenceRight = encoder2.getPosition() - rotationsInitRight;
    
    if (rollAngleDegrees != 0.0) {
      double radiansPerAngle = slope * rollAngleDegrees;
      SmartDashboard.putNumber("radiansPerAngle", radiansPerAngle);

      double direction = 1.0;
      if (rollAngleDegrees < 0.0) {
        direction = -1.0;
      }

      SmartDashboard.putNumber("direction", direction);

      radiansPerAngle = radiansPerAngle + (minMovementSpeed * direction);
      SmartDashboard.putNumber("radiansPerAngle2", radiansPerAngle);
      try {
        double encoder0Position = encoder0.getPosition();
        double encoder2Position = encoder2.getPosition();

        SmartDashboard.putNumber("enc0Pos", encoder0Position);
        SmartDashboard.putNumber("enc2Pos", encoder2Position);
        SmartDashboard.putNumber("rotationsInitLeft", rotationsInitLeft);
        SmartDashboard.putNumber("rotationinitRight", rotationsInitRight);
        SmartDashboard.putNumber("Difference left", differenceLeft);
        SmartDashboard.putNumber("Difference right", differenceRight);

        double leftSpeed;
        if (Math.abs(differenceLeft) >= rotationsToBalance) {
          leftSpeed = radiansPerAngle - (brakeAdjustment * direction);
          leftSpeed = 0.0;
        } else {
          leftSpeed = radiansPerAngle;
        }


        double rightSpeed;
        if (Math.abs(differenceRight) >= rotationsToBalance) {
          rightSpeed = radiansPerAngle - (brakeAdjustment * direction);
          rightSpeed=0.0;
        } else {
          rightSpeed = radiansPerAngle;
        }

        drive(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("rightSpeed", rightSpeed);
        SmartDashboard.putNumber("leftSpeed", leftSpeed);

      } catch(RuntimeException ex) {
        String err_string = "Drive system error: " + ex.getMessage();
        DriverStation.reportError(err_string, true);
      }

      Timer.delay(0.005);
    }
  }

     private void moveForwardOrBack(int distanceInInches, double speed) {
        // Get current motor positions

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


    private void strafeLeftOrRight (int distanceInInches, double speed){ 
    double motorController0Position = encoder0.getPosition();
    double motorController2Position = encoder2.getPosition();
    double movement = distanceInInches * rotationsPerInch;
    
    motorController0Position += movement;
    motorController2Position -= movement;


    encoder0.setPosition(motorController0Position);
    encoder2.setPosition(motorController2Position);

    motorController00.set(speed);
    motorController02.set(speed);

    while (encoder0.getPosition() <= motorController0Position && encoder2.getPosition() <= motorController2Position){

      encoder0.setPosition(motorController0Position);
      encoder2.setPosition(motorController2Position);

    }
    
  }


    /** 
     * rotateLeftOrRight()
     * 
     * Rotate angle left or right
     * 
     * @params int rotateAngle Postive value rotates right, negative rotates left
     * @params double speed Range 0.0 to 1.0
     */
    /*private void rotateLeftOrRight(int rotateAngle, double speed) {

        // Get current motor positions
        double motorController0Position = encoder0.getPosition();
        double motorController2Position = encoder2.getPosition();
        double movement = rotateAngle * movementPerDegree;

        // Apply movement
        motorController0Position += movement;
        motorController2Position -= movement;

        // Set location to move to
        encoder0.setPosition(motorController0Position);
        encoder2.setPosition(motorController2Position);
        
        // Start moving robot by setting motor speed
        motorController00.set(speed);
        motorController02.set(speed);
        
        while (encoder0.getPosition() <= motorController0Position && encoder2.getPosition() <= motorController2Position){

          encoder0.setPosition(motorController0Position);
          encoder2.setPosition(motorController2Position);

        }

        motorController00.set(0.0);
        motorController02.set(0.0);

    };*/

}

