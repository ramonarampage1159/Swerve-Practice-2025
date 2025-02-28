// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class CoralCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CoralCommand() {
   // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_coralSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_leftBumper)){
      RobotContainer.m_coralSubsystem.IntakeCoral();
    }else if (RobotContainer.m_operatorController.getRawButtonReleased(Constants.OperatorConstants.JoystickButtons.m_leftBumper)){
      RobotContainer.m_coralSubsystem.stopMotors();
    }


    if (RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_leftTrigger)){
      RobotContainer.m_coralSubsystem.shootCoral();
    }else if (RobotContainer.m_operatorController.getRawButtonReleased(Constants.OperatorConstants.JoystickButtons.m_leftTrigger)){
      RobotContainer.m_coralSubsystem.stopMotors();
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
