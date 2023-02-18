// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  public CANSparkMax indexercontroller;
  /** Creates a new Indexer. */ 

  public IndexerSubsystem() {
    indexercontroller = new CANSparkMax(10, MotorType.kBrushless);
  }

  public void forward (){
    indexercontroller.set(0.5);
  }
  public void backward (){
indexercontroller.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}