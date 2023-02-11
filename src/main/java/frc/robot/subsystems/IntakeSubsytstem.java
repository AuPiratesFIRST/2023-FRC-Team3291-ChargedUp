// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsytstem extends SubsystemBase {

  public double position0 = 0;
  //find the actual vaue later when you can. 
  //I feel like using the length of the intake doubled as the diameter could work, 
  //and then you use the same formula for the wheels? Just thinking about circles.
  public double position1 = 60;

  //I think I can use a Talon but Mr.Cameron says it'll be a Victor, so let's just creat two versions, bam
  public VictorSPX flipper;

  public CANSparkMax intakeController0;

  public CANSparkMax intakeController1; 

  DigitalInput flipSwitch = new DigitalInput(0);
  DigitalInput objectSwitch = new DigitalInput(1);  

  RelativeEncoder flipEncoder;

  double wheelPower = Constants.Intake.wheelPower;

  //public CANSparkMax intakeController0;

  /** Creates a new IntakeSubsytstem. */
  public IntakeSubsytstem() {
    flipper = new VictorSPX(45);

    intakeController0 = new CANSparkMax(
      Constants.Intake.canMotorDeviceId05,
      MotorType.kBrushless
    );

    intakeController1 = new CANSparkMax(
      Constants.Intake.canMotorDeviceId06,
      MotorType.kBrushless
    );

    intakeController0.setInverted(true);

    //public RelativeEncoder flipEncoder = flipper.getEncoder()

  }

  public void forward() {
    intakeController0.set(wheelPower);
    intakeController1.set(wheelPower);
  
  }

  public void backward() {
    intakeController0.set(wheelPower);
    intakeController1.set(wheelPower);

  }

  public void flipUp(){

    flipper.set(ControlMode.Position, position1);
    flipper.set(VictorSPXControlMode.PercentOutput, 0.5);
  }

  public void flipDown(){

    if (flipEncoder.getPosition() >= position1){
      flipper.set(VictorSPXControlMode.PercentOutput, 0.0);
      intakeController0.set(wheelPower);
      intakeController1.set(wheelPower);

      flipper.set(ControlMode.Position, position0);
      flipper.set(VictorSPXControlMode.PercentOutput, -0.5);
    } 

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
