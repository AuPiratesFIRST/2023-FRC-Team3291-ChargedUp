// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public CANSparkMax intakeController0;
  
  double wheelPower = Constants.Intake.wheelPower;

  //public CANSparkMax intakeController0;

  /** Creates a new IntakeSubsytstem. */
  public IntakeSubsystem() {

    intakeController0 = new CANSparkMax(
      Constants.Intake.canMotorDeviceId05,
      MotorType.kBrushless
    );

  }

  public void forward() {
    intakeController0.set(wheelPower);
  }

  public void backward() {
    intakeController0.set(-wheelPower);
  }




  /*public void flip2(){

    if(objectSwitch.get()){

      flipper.set(ControlMode.Position, position0);
      flipper.set(VictorSPXControlMode.PercentOutput, 0.5);

      if(flipSwitch.get()){

        intakeController0.set(-0.5);
        intakeController1.set(0.5);
        //find a way to make it continue for 1 second without throwing an error
      
        flipper.set(ControlMode.Position, position0);
        flipper.set(VictorSPXControlMode.PercentOutput, 0.5);

    }

    } else {
      flipper.set(ControlMode.Position, position0);
      flipper.set(VictorSPXControlMode.PercentOutput, 0);
    }

  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
