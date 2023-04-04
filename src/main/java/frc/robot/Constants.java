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
   public final static int canMotorDeviceId01 = 11; //Front left
    public final static int canMotorDeviceId02 = 13; //Back left
    public final static int canMotorDeviceId03 = 14; //Front rgiht
    public final static int canMotorDeviceId04 = 12; //Back right

    
    /*public final static int canMotorDeviceId01 = 7; //Front left
    public final static int canMotorDeviceId02 = 8; //Back left
    public final static int canMotorDeviceId03 = 6; //Front rgiht
    public final static int canMotorDeviceId04 = 5; //Back right*/


    public final static double kPDrive = 0.009;
    public final static double kIDrive = 0.0;
    public final static double kDDrive = 0.09;

    /*****
     * Custom DriveTrain
     */
    public static final double kDefaultDeadband = 0.1;
    public static final double kDefaultMaxOutput = 0.3;

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

    public final static int canMotorDeviceId05 = 10;
    public final static int canMotorDeviceId06 = 15;
    public final static double intakeSpeed = 0.1;
    public final static double intakeSpeedCone = 0.5;
    public final static double intakeSpeedCube = 0.3;
  }

  public final class Indexer {

    public final static int canMotorPort = 44;
    public final static double indexerSpeed = 0.2;
    public final static double indexerConeSpeed = 0.2;
    public final static double indexerCubeSpeed = 0.1;
  }
  public final class Joystick {
    public final static int tankLeftPort = 0;
    public final static int tankRightPort = 1;
    public final static int secondDriver = 2;
    public final static double deadband = 0.1;
  }

  public final static class Lighting {
    public final static int lightingPort = 1;

    public enum Colors {
      // HOTPINK   ("Hot Pink", 0.57),
      // DARKRED   ("Dark Red", 0.59),
      RED       ("Red", 0.61),
      // REDORANGE ("Red Orange", 0.63),
       // ORANGE    ("Orange", 0.65),
      // GOLD      ("Gold", 0.67),
      // YELLOW    ("Yellow", 0.69),
      // LAWNGREEN ("Lawn Green", 0.71),
      // LIME      ("Lime", 0.73),
      // DARKGREEN ("Dark Green", 0.75),
      // GREEN     ("Green", 0.77),
      // BLUEGREEN ("Blue Green", 0.79),
      // AQUA      ("Aqua", 0.81),
      // SKYBLUE   ("Sky Blue", 0.83),
      // DARKBLUE  ("Dark Blue", 0.85),
      BLUE      ("Blue", 0.87),
      // BLUEVIOLET ("Blue Violet", 0.89),
      // VIOLET    ("Violet", 0.91),
      // WHITE     ("White", 0.93),
      // GRAY      ("Gray", 0.95),
      // DARKGRAY  ("Dark Gray", 0.97),
      RAINBOW   ("Rainbow", -0.99),
      // RAINBOWGLITTER ("Rainbow- Glitter", -0.89),
      // RAINBOWSINELON ("Rainbow - Sinelon", -0.79),
      // RAINBOWBEATS ("Rainbow - Beats Per Minute", -0.69),
      // RAINBOWTWINKLES ("Rainbow - Tinkles", -0.55),
      // RAINBOWWAVES ("Rainbow - Color Waves", -0.45),
      // REDCHASE  ("Light Chase - Red", -0.31),
      // BLUECHASE ("Light Chase - Blue", -0.29),
      OFF       ("Off", 0.99);
      
      public final String colorName;
      public final double colorValue;

      Colors(String colorName, double colorValue) {
        this.colorName = colorName;
        this.colorValue = colorValue;
      } 

      public String getColorName() {
        return this.colorName;
      }

      public double getColorValue() {
        return this.colorValue;
      }
    }

    public final static Colors startingColor = Colors.OFF;
  }  

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final static double STOPPOWER = 0.0;

  
}
