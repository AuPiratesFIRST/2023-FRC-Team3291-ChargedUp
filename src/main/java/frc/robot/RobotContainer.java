// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.autonomous.Auto00;
import frc.robot.commands.autonomous.Auto1;
import frc.robot.commands.autonomous.Auto1vr;
import frc.robot.commands.autonomous.Auto2;
import frc.robot.commands.autonomous.Auto4;
import frc.robot.commands.autonomous.Auto5;
import frc.robot.commands.autonomous.Auto6;
import frc.robot.commands.indexer.IndexerBackward;
import frc.robot.commands.indexer.IndexerFoward;
import frc.robot.commands.intake.IntakeBackward;
import frc.robot.commands.intake.IntakeForward;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final Command AutoBalanceCommand = null;
  // The robot's subsystems and commands are defined here...
  private DriveTrainSubsystems driveTrainSubsystem = new DriveTrainSubsystems();
  private IndexerSubsystem indexsubsystem = new IndexerSubsystem();
  private IntakeSubsystem intakeSubsytstem = new IntakeSubsystem();
  public LightingSubsystem lightingSubsystem = new LightingSubsystem();

  public CommandJoystick controller00 = new CommandJoystick(0);
  public CommandJoystick joystick00 = new CommandJoystick(Constants.Joystick.tankLeftPort);
  public CommandJoystick joystick01 = new CommandJoystick(Constants.Joystick.tankRightPort);
  public CommandJoystick joystick02 = new CommandJoystick(Constants.Joystick.secondDriver);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private IntakeForward intakeForward = new IntakeForward(intakeSubsytstem);

  private AutobalanceCommand autobalanceCommand = new AutobalanceCommand(driveTrainSubsystem);

  private IndexerFoward indexerFowardConeCommand = new IndexerFoward(indexsubsystem);
  private IndexerBackward indexerBackwardConeCommand = new IndexerBackward(indexsubsystem);
  private IntakeForward intakeForwardConecommand = new IntakeForward(intakeSubsytstem);
  private IntakeBackward intakeBackwardConecommand = new IntakeBackward(intakeSubsytstem);

  private IndexerFoward indexerFowardCubeCommand = new IndexerFoward(indexsubsystem);
  private IndexerBackward indexerBackwardCubeCommand = new IndexerBackward(indexsubsystem);
  private IntakeForward intakeForwardCubecommand = new IntakeForward(intakeSubsytstem);
  private IntakeBackward intakeBackwardCubecommand = new IntakeBackward(intakeSubsytstem);
  
  //private IntakeAndIndexer intakeAndIndexer = new IntakeAndIndexer(driveTrainSubsystem, indexsubsystem, intakeSubsytstem);

  private Auto00 auto00 = new Auto00();
  private Auto1 auto01 = new Auto1(driveTrainSubsystem, indexsubsystem, intakeSubsytstem);
  private Auto2 auto02 = new Auto2(driveTrainSubsystem, indexsubsystem);
  private Auto4 auto04 = new Auto4(driveTrainSubsystem, indexsubsystem, intakeSubsytstem);
  private Auto5 auto05 = new Auto5(driveTrainSubsystem, indexsubsystem, intakeSubsytstem);
  private Auto6 auto06 = new Auto6(driveTrainSubsystem, indexsubsystem, intakeSubsytstem);
  private Auto1vr auto1vr = new Auto1vr(driveTrainSubsystem);

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  /*private Command IndexerBackwardCone;
  private Command IndexerForwardCone;
  private Command IndexerBackwardCube;
  private Command IndexerForwardCube;

  private Command intakeBackwardCone;
  private Command intakeBackwardCube;
  private Command intakeForwardCone;
  private Command intakeForwardCube;*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //driveTrainSubsystem.setDefaultCommand(new Drive(driveTrainSubsystem, () -> joystick00.getX(), () ->  joystick01.getX()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}
   */
  private void configureBindings() {
  
    m_Chooser.setDefaultOption("Autonomous 00", auto00);
    m_Chooser.addOption("Autonomous 01", auto01);
    m_Chooser.addOption("Autonomous 02", auto02);
    m_Chooser.addOption("Autonomous 04", auto04);
    m_Chooser.addOption("Autonomous 05", auto05);
    m_Chooser.addOption("Autonomous 06", auto06);
    m_Chooser.addOption("Autonomous 1vr", auto1vr);
    SmartDashboard.putData("Auto choices", m_Chooser);

    /*
    driveTrainSubsystem.setDefaultCommand(
      new RunCommand(
        () ->
        driveTrainSubsystem.drive(
          controller00.getRawAxis(1),
          controller00.getRawAxis(3)
        ),
      driveTrainSubsystem)
    );
    */
  
    driveTrainSubsystem.setDefaultCommand(
      new RunCommand(
        () ->
        driveTrainSubsystem.drive(
          joystick00.getRawAxis(1),
          joystick01.getRawAxis(1)
        ),
      driveTrainSubsystem)
    );

    joystick02.button(1).whileTrue(autobalanceCommand);

    joystick02.button(7).whileTrue(intakeForward);
    joystick02.button(8).whileTrue(intakeBackwardConecommand);
    joystick02.button(9).whileTrue(indexerFowardConeCommand);
    joystick02.button(10).whileTrue(indexerBackwardConeCommand);
   
    joystick02.button(11).whileTrue(indexerFowardCubeCommand);
    joystick02.button(12).whileTrue(indexerBackwardCubeCommand);
    joystick02.button(3).whileTrue(intakeForwardCubecommand);
    joystick02.button(4).whileTrue(intakeBackwardCubecommand);

    /*if (MathUtil.applyDeadband(joystick02.getRawAxis(1), Constants.Joystick.deadband)>0){
      intakeBackwardcommand.schedule();
    } else {
      indexerBackwardCommand.cancel();
    }

    if (MathUtil.applyDeadband(joystick02.getRawAxis(1), Constants.Joystick.deadband)<0){
      intakeBackwardcommand.schedule();
    } else {
      indexerBackwardCommand.cancel();
    }
    */
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_Chooser.getSelected();
  }
}

