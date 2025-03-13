
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 // private final ExampleSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeCommand() {
   // m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_AlgaeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if (RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_rightBumper)){
      RobotContainer.m_AlgaeSubsystem.IntakeAlgae();
    }else if (RobotContainer.m_operatorController.getRawButtonReleased(Constants.OperatorConstants.JoystickButtons.m_rightBumper)){
      RobotContainer.m_AlgaeSubsystem.stopMotors();
    }

    if (RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_rightTrigger)){
      RobotContainer.m_AlgaeSubsystem.shootAlgae();
    }else if (RobotContainer.m_operatorController.getRawButtonReleased(Constants.OperatorConstants.JoystickButtons.m_rightTrigger)){
      RobotContainer.m_AlgaeSubsystem.stopMotors();
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
