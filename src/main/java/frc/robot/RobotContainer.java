// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static boolean m_FieldCentricMode = true;

  public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();
  public static ManualElevatorSubsystem m_manualElevatorSubsystem = new ManualElevatorSubsystem();
  public static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  public static Joystick m_driverController = new Joystick (Constants.DriverConstants.m_driverController);
  public static Joystick m_operatorController = new Joystick(Constants.OperatorConstants.m_operatorController);

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("Hello World"));

  }


  public Command driveDirectAngle = m_swerveDriveSubsystem.driveCommand(
     () -> MathUtil.applyDeadband(m_driverController.getRawAxis(1), DriverConstants.translationDEADBAND),
     () -> MathUtil.applyDeadband(m_driverController.getRawAxis(0), DriverConstants.translationDEADBAND),
     () -> MathUtil.applyDeadband(m_driverController.getRawAxis(2), DriverConstants.rotationDEADBAND)); 

      
  public Command driveRobotOrientedDirectAngle = m_swerveDriveSubsystem.driveRobotCentricCommand(
     () -> MathUtil.applyDeadband(m_driverController.getRawAxis(1) , DriverConstants.translationDEADBAND),
     () -> MathUtil.applyDeadband(m_driverController.getRawAxis(0), DriverConstants.translationDEADBAND),
     () -> MathUtil.applyDeadband(m_driverController.getRawAxis(2), DriverConstants.rotationDEADBAND));

      
  public boolean getRawButtonPressed(int button){
      return m_operatorController.getRawButton(button);
  }
    

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {

    m_swerveDriveSubsystem.setDefaultCommand(driveDirectAngle);
    m_manualElevatorSubsystem.setDefaultCommand(new ManualElevatorCommand());
    m_elevatorSubsystem.setDefaultCommand(new ElevatorCommand());

    new JoystickButton(m_driverController, Constants.DriverConstants.JoystickButtons.m_xButton).onTrue(driveDirectAngle); 
    new JoystickButton(m_driverController, Constants.DriverConstants.JoystickButtons.m_aButton).onTrue(driveRobotOrientedDirectAngle);

    if(m_driverController.getRawButtonPressed(Constants.DriverConstants.JoystickButtons.m_bButton)) {
      m_swerveDriveSubsystem.zeroGyro();
    }

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return m_swerveDriveSubsystem.getAutonomousCommand("New Auto");
    return autoChooser.getSelected();
  }

}