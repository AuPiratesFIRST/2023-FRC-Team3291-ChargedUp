// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  public TalonFX indexercontroller;
  /** Creates a new Indexer. */ 

  public IndexerSubsystem() {
    indexercontroller = new TalonFX(Constants.Indexer.canMotorPort);
  }

  public void forward (){
    indexercontroller.set(ControlMode.PercentOutput, 0.5);
  }
  public void backward (){
indexercontroller.set(ControlMode.PercentOutput, -0.5);
  }

  public void stop (){
    indexercontroller.set(ControlMode.PercentOutput, Constants.STOPPOWER);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}