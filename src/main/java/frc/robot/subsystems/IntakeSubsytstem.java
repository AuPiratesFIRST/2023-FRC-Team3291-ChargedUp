// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsytstem extends SubsystemBase {
public TalonFX intakecontroller;

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
