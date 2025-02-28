package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ManualElevatorCommand extends Command {
  /** Creates a new ManualElevatorCommand. */

  public ManualElevatorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  //addRequirements(RobotContainer.m_manualElevatorSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*
     SmartDashboard.putNumber("Elevator Motor Current", RobotContainer.m_manualElevatorSubsystem.getCurrentValue());

    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_xButton)){

      if(RobotContainer.m_manualElevatorSubsystem.getCurrentValue() <= Constants.ElevatorConstants.MAX_CURRENT){
        RobotContainer.m_manualElevatorSubsystem.ElevatorUp();
      }
      else{
        RobotContainer.m_manualElevatorSubsystem.stopMotors();
      }

    }else if(RobotContainer.m_operatorController.getRawButtonReleased(Constants.OperatorConstants.JoystickButtons.m_xButton)){
      RobotContainer.m_manualElevatorSubsystem.stopMotors();

    }else if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_aButton)){
      RobotContainer.m_manualElevatorSubsystem.ElevatorDown();

    }else if(RobotContainer.m_operatorController.getRawButtonReleased(Constants.OperatorConstants.JoystickButtons.m_aButton)){
      RobotContainer.m_manualElevatorSubsystem.stopMotors();
    }   
   


  */
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