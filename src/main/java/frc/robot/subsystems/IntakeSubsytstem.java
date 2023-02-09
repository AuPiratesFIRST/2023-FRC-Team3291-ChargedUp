// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

public class IntakeSubsytstem extends SubsystemBase {
public TalonFX intakecontroller;

public CANSparkMax motorController00 = new CANSparkMax(
  Constants.Intake.canMotorDeviceId05,
  MotorType.kBrushless
);

public CANSparkMax motorController01 = new CANSparkMax(
  Constants.Intake.canMotorDeviceId06,
  MotorType.kBrushless
);

  /** Creates a new IntakeSubsytstem. */
  public IntakeSubsytstem() {

    intakecontroller = new TalonFX(45);
    
  }

  public void forward() {
    intakecontroller.set(ControlMode.PercentOutput, 0.5);
  }

  public void backward() {
    intakecontroller.set(ControlMode.PercentOutput, -0.5);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
