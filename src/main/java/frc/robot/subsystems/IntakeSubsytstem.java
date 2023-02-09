// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.platform.can.PlatformCAN;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
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

public class IntakeSubsytstem extends SubsystemBase {
public VictorSPX flipper;

public CANSparkMax intakeController0;

public CANSparkMax intakeController1; 

DigitalInput flipSwitch = new DigitalInput(0);
DigitalInput objectSwitch = new DigitalInput(1);

//public CANSparkMax intakeController0;

  /** Creates a new IntakeSubsytstem. */
  public IntakeSubsytstem() {
    flipper = new TalonFX(45);

    intakeController0 = new CANSparkMax(
      Constants.DriveTrain.canMotorDeviceId01,
      MotorType.kBrushless
    );

    intakeController1 = intakeController0 = new CANSparkMax(
      Constants.DriveTrain.canMotorDeviceId01,
      MotorType.kBrushless
    );

    

  }

  public void forward() {
    intakeController0.set(0.5);
    intakeController1.set(-0.5);
  
  }

  public void backward() {
    intakeController0.set(-0.5);
    intakeController1.set(0.5);

  }

  public void flip(){

    if(flipSwitch.get()){

      flipper.

    }

  }

  public void flip2(){

    if(objectSwitch.get()){

      if(flipSwitch)

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
