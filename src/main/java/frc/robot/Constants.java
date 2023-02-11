// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public final class DriveTrain {
    public final static int canMotorDeviceId01 = 7;
    public final static int canMotorDeviceId02 = 8;
    public final static int canMotorDeviceId03 = 6;
    public final static int canMotorDeviceId04 = 5;

    /*****
     * Custom DriveTrain
     */
    public static final double kDefaultDeadband = 0.03;
    public static final double kDefaultMaxOutput = 1.0;

    /******
     * Autobalance
     */
    public final static double wheelDiameter = 8.0;
    public final static double platformWidth = 48.0;
    public final static double robotLength = 28.0;
    public final static double minAngle = 0.0;
    public final static double maxAngle = 15.0;
    public final static double minMovementSpeed = 0.3;
    public final static double maxMovementSpeed = 0.7;
    public final static double brakeAdjustment = 0.05;
  }

  public final class Intake {

    public final static int canMotorDeviceId05 = 9;
    public final static int canMotorDeviceId06 = 10;

  }

  public final class Joystick {
    public final static int tankLeftPort    = 0;
    public final static int tankRightPort   = 1;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final static double STOPPOWER = 0.0;

  
}
