// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
//new
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDriveSubsystem driveBase = new SwerveDriveSubsystem();

  //  private final SwerveDriveSubsystem driveBase = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));


  public static Joystick  m_driverController = new Joystick (OperatorConstants.driverController);
  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientedDirectAngle);
    //driveBase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngleKeyboard : driveFieldOrientedDirectAngleKeyboard);
    //driveBase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleKeyboard);


  /* // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController  m_driverController =
      new CommandXboxController (OperatorConstants.kDriverControllerPort);
  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveBase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngleKeyboard : driveFieldOrientedDirectAngleKeyboard);
    //driveBase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleKeyboard);*/
    
    
  }


 /*SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                                                             .headingWhile(true);

  Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);

  Command drievFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);*/

  //Test
  //Command drievFieldOrientedAngularVelocityTest = driveBase.driveFieldOrientedTest(driveAngularVelocity,m_driverController.getLeftX(),m_driverController.getLeftY(),m_driverController.getRightX());

  

 //WORKING TODAY 02/11
 /* SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(driveBase.getSwerveDrive(), 
                                                                        () -> -m_driverController.getRawAxis(1) *-1 ,//getLeftY() * -1,  //works the same as
                                                                        () -> -m_driverController.getRawAxis(0) *-1)//getLeftX() *-1)    //works the same as
                                                                        .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                                        .deadband(OperatorConstants.DEADBAND)
                                                                        .scaleTranslation(0.8)
                                                                        .allianceRelativeControl(true);
 // Derive the heading axis with math!



SwerveInputStream driveDirectAngleKeyboard  = driveAngularVelocityKeyboard.copy()
.withControllerHeadingAxis(() -> 
                              m_driverController.getRawAxis(
                                2),
                           () -> 
                              m_driverController.getRawAxis(
                                 3) )
 .headingWhile(false); //CY wanted to not use this line so changed to false 
 
 Command driveFieldOrientedDirectAngleKeyboard = driveBase.driveFieldOriented(driveDirectAngleKeyboard);
  //new, seems like made no difference
  Command drievFieldOrientedAngularVelocityKeyboard = driveBase.driveFieldOriented(driveAngularVelocityKeyboard);
  //Command driveSetpointGenKeyboard = driveBase.drievWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);   
 
 
 */



 // 02/11 NOT USED 
 /*  SwerveInputStream driveDirectAngleKeyboard  = driveAngularVelocityKeyboard.copy()
                                                                            .withControllerHeadingAxis(() -> Math.sin(
                                                                                                          m_driverController.getRawAxis(
                                                                                                            2) * Math.PI) * (Math.PI *2),
                                                                                                       () -> Math.cos(
                                                                                                          m_driverController.getRawAxis(
                                                                                                             2) * Math.PI) *
                                                                                                            (Math.PI * 2))
                                                                             .headingWhile(true); */



  
     Command driveFieldOrientedDirectAngle = driveBase.driveCommand(
       () -> MathUtil.applyDeadband(m_driverController.getRawAxis(1), OperatorConstants.DEADBAND),
       () -> MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.DEADBAND),
       () -> m_driverController.getRawAxis(2)); 




  

                          




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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //new removed because XBox
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
