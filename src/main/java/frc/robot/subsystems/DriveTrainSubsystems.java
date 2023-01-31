// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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
  
  public MotorController motorControler00 = new CANSparkMax(
    Constants.DriveTrain.canMotorDeviceId01,
    MotorType.kBrushless
  );

  public MotorController motorController01 = new CANSparkMax(
    Constants.DriveTrain.canMotorDeviceId02,
    MotorType.kBrushless
  );

  public MotorController motorController02 = new CANSparkMax(
  Constants.DriveTrain.canMotorDeviceId03,
  MotorType.kBrushless 
  );

  private MotorControllerGroup leftMotors = new MotorControllerGroup(
    this.motorControler00,
    this.motorController01
  );

  private MotorControllerGroup rightMotors = new MotorControllerGroup(
    this.motorController02,
    this.motorController01   
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

    double xPower = rollAngleDegrees * (0.4/11) + 0.25; 
    double zPower = pitchAngleDegrees * (0.4/11) + 0.25; 
 
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
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      xAxisRate = ((Math.sin(rollAngleRadians) * -1) * (xPower))/100;
      if (xAxisRate > 0.7){
        xAxisRate = 0.7;
      if (xAxisRate < 0.3){
        xAxisRate = 0.3;
      }
      }
    }

    if ( autoBalanceYMode ) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      yAxisRate = ((Math.sin(pitchAngleRadians) * -1) * (zPower))/100;
      if (yAxisRate > 0.7){
        yAxisRate = 0.7;
      }
      if (yAxisRate< 0.3){
        yAxisRate = 0.3;
      }
    }

    SmartDashboard.putNumber("X axis rate", xAxisRate);
    SmartDashboard.putNumber("Y axis rate", yAxisRate);
    SmartDashboard.putNumber("zPower", zPower);
    SmartDashboard.putNumber("xPower", xPower);
    SmartDashboard.putBoolean("Autobalance Y mode", autoBalanceYMode);
    SmartDashboard.putBoolean("Autobalance X mode", autoBalanceXMode);
    SmartDashboard.putNumber("Roll angle degrees", rollAngleDegrees);
    SmartDashboard.putNumber("Pitch angle degrees", pitchAngleDegrees);

    try {
       m_drive.arcadeDrive(yAxisRate, xAxisRate);

    } catch(RuntimeException ex) {
      String err_string = "Drive system error:  " + ex.getMessage();
      DriverStation.reportError(err_string, true);
      status = false;
    }
    
    Timer.delay(0.005);

    return status;
  }

}
