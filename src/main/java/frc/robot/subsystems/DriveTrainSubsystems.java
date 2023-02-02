// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.platform.can.PlatformCAN;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import com.revrobotics.RelativeEncoder;


public class DriveTrainSubsystems extends SubsystemBase {
  public final static int RIGHT = 1;
  public final static int LEFT = -1;


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
  public RelativeEncoder encoder1 = motorController01.getEncoder();
  public RelativeEncoder encoder2 = motorController02.getEncoder();
  public RelativeEncoder encoder3 = motorController03.getEncoder();

  private MotorControllerGroup leftMotors = new MotorControllerGroup(
    this.motorController00,
    this.motorController01
  );

  private MotorControllerGroup rightMotors = new MotorControllerGroup(
    this.motorController02,
    this.motorController03   
  );

  DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);  

  /** Creates a new DriveTrainSubsystems. */
  public DriveTrainSubsystems() {
    
    leftMotors.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
  
    double speed = 0.70;

    leftSpeed = leftSpeed * speed;
    rightSpeed = rightSpeed * speed;

    SmartDashboard.putNumber("leftSpeed", leftSpeed);
    SmartDashboard.putNumber("rightSpeed", rightSpeed);

    m_drive.tankDrive(leftSpeed, rightSpeed);

  }

  public boolean autoBalance() {
    boolean status = true;

    double xAxisRate = 0.0;
    double yAxisRate = 0.0; 

    // Retrieve robot angle in 3D space
    double pitchAngleDegrees = navx_device.getPitch();
    double rollAngleDegrees = navx_device.getRoll();

    double diameter = 8;
    double platformWidth = 48;
    double robotLength = 28;
    double minAngle = 0;
    double minSpeedFlat = 0.3;
    double maxAngle = 15;
    double minSpeedMax = 0.7;
    //double breakAdj = 0.05;

    double rotationsPerInch = 1/(diameter * Math.PI);
    double distanceToEdge =  platformWidth - (platformWidth - robotLength)/2;
    double rotationsToBalance =  distanceToEdge * rotationsPerInch;
    double YradiansPerAngle = (((((minSpeedMax - minSpeedFlat)/(maxAngle - minAngle)) * rollAngleDegrees)+minSpeedFlat) * -1);
    double XradiansPerAngle = (((minSpeedMax - minSpeedFlat)/(maxAngle - minAngle)) * pitchAngleDegrees)+minSpeedFlat;
    double rotationsNeeded = encoder0.getPosition()+ rotationsToBalance;
    //double radiansToBrake = XradiansPerAngle - breakAdj;

    //double xPower = rollAngleDegrees * (0.4/11) + 0.25; 
    //double zPower = pitchAngleDegrees * (0.4/11) + 0.25; 
 
    if ( !autoBalanceXMode && 
      (Math.abs(rollAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))
    ) {
      autoBalanceXMode = true;
    } else if ( autoBalanceXMode && 
      (Math.abs(rollAngleDegrees) <= 
        Math.abs(kOonBalanceAngleThresholdDegrees))
    ) {
      autoBalanceXMode = false;
    }

    if ( !autoBalanceYMode && 
      (Math.abs(pitchAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))
    ) {
      autoBalanceYMode = true;
    } else if ( autoBalanceYMode && 
      (Math.abs(pitchAngleDegrees) <= 
        Math.abs(kOonBalanceAngleThresholdDegrees))
    ) {
      autoBalanceYMode = false;
    }

    /* 
     * Control drive system automatically, 
     * driving in reverse direction of pitch/roll angle,
     * with a magnitude based upon the angle
     */
    if ( autoBalanceXMode ) {
      //double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      //xAxisRate = Math.sin(rollAngleRadians);
      encoder0.setPosition(rotationsNeeded);
      encoder1.setPosition(rotationsNeeded);
      encoder2.setPosition(rotationsNeeded);
      encoder3.setPosition(rotationsNeeded);
      if (xAxisRate > 0.7){
        xAxisRate = 0.7;
      if (xAxisRate < 0.3){
        xAxisRate = 0.3;
      }
      }
    }

    if ( autoBalanceYMode ) {
      //double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      encoder0.setPosition(rotationsNeeded);
      encoder1.setPosition(rotationsNeeded);
      encoder2.setPosition(rotationsNeeded);
      encoder3.setPosition(rotationsNeeded);
      if (yAxisRate > 0.7){
        yAxisRate = 0.7;
      }
      if (yAxisRate< 0.3){
        yAxisRate = 0.3;
      }
    }

    SmartDashboard.putNumber("X axis rate", xAxisRate);
    SmartDashboard.putNumber("Y axis rate", yAxisRate);
    SmartDashboard.putBoolean("Autobalance Y mode", autoBalanceYMode);
    SmartDashboard.putBoolean("Autobalance X mode", autoBalanceXMode);
    SmartDashboard.putNumber("Roll angle degrees", rollAngleDegrees);
    SmartDashboard.putNumber("Pitch angle degrees", pitchAngleDegrees);

    try {
       m_drive.arcadeDrive(YradiansPerAngle, XradiansPerAngle);

    } catch(RuntimeException ex) {
      String err_string = "Drive system error:  " + ex.getMessage();
      DriverStation.reportError(err_string, true);
      status = false;
    }
    
    Timer.delay(0.005);

    return status;
  }

}
