package frc.robot.commands;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
 


public class ElevatorCommand extends Command{
    public ElevatorCommand(){
      addRequirements(RobotContainer.m_elevatorSubsystem);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //SmartDashboard.putNumber("Elevator Current TEST", RobotContainer.m_elevatorSubsystem.getCurrentValue());

    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_xButton)) {
      double L1PValue = Constants.ElevatorConstants.L1_PIDS.m_L1P;
      double L1IValue = Constants.ElevatorConstants.L1_PIDS.m_L1I;
      double L1DValue = Constants.ElevatorConstants.L1_PIDS.m_L1D;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L1PValue, L1IValue, L1DValue, 
      Constants.ElevatorConstants.L1_PIDS.m_L1MinOutput, Constants.ElevatorConstants.L1_PIDS.m_L1MaxOutput);
      double L1Rotations = Constants.ElevatorConstants.L1_PIDS.m_L1Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L1Rotations, SparkBase.ControlType.kPosition);
    }
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_bButton)) {
      double L2PValue = Constants.ElevatorConstants.L2_PIDS.m_L2P;
      double L2IValue = Constants.ElevatorConstants.L2_PIDS.m_L2I;
      double L2DValue = Constants.ElevatorConstants.L2_PIDS.m_L2D;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L2PValue, L2IValue, L2DValue, 
      Constants.ElevatorConstants.L2_PIDS.m_L2MinOutput, Constants.ElevatorConstants.L2_PIDS.m_L2MaxOutput);
      double L2Rotations = Constants.ElevatorConstants.L2_PIDS.m_L2Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L2Rotations, SparkBase.ControlType.kPosition);
    }
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_yButton)) {
      double L3PValue = Constants.ElevatorConstants.L3_PIDS.m_L3P;
      double L3IValue = Constants.ElevatorConstants.L3_PIDS.m_L3I;
      double L3DValue = Constants.ElevatorConstants.L3_PIDS.m_L3D;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L3PValue, L3IValue, L3DValue, 
      Constants.ElevatorConstants.L3_PIDS.m_L3MinOutput, Constants.ElevatorConstants.L3_PIDS.m_L3MaxOutput);
      double L3Rotations = Constants.ElevatorConstants.L3_PIDS.m_L3Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L3Rotations, SparkBase.ControlType.kPosition); 
    }
    /*
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_aButton)) {
      double ZeroPValue = Constants.ElevatorConstants.ZeroPIDS.m_ZerokP;
      double ZeroIValue = Constants.ElevatorConstants.ZeroPIDS.m_ZerokI;
      double ZeroDValue = Constants.ElevatorConstants.ZeroPIDS.m_ZerokD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(ZeroPValue, ZeroIValue, ZeroDValue, 
      Constants.ElevatorConstants.ZeroPIDS.m_ZerokMinOutput, Constants.ElevatorConstants.ZeroPIDS.m_ZerokMaxOutput);
      double ZeroRotations = Constants.ElevatorConstants.ZeroPIDS.m_ZerokRotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(ZeroRotations, SparkBase.ControlType.kPosition);
    }
      */
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_aButton)) {
      double ProcessorPValue = Constants.ElevatorConstants.Processor_PIDS.m_processorP;
      double ProcessorIValue = Constants.ElevatorConstants.Processor_PIDS.m_processorI;
      double ProcessorDValue = Constants.ElevatorConstants.Processor_PIDS.m_processorD;

      RobotContainer.m_elevatorSubsystem.setPIDValues(ProcessorPValue, ProcessorIValue, ProcessorDValue, 
      Constants.ElevatorConstants.Processor_PIDS.m_processorMinOutput, Constants.ElevatorConstants.Processor_PIDS.m_processorMaxOutput);
      double ProcessorRotations = Constants.ElevatorConstants.Processor_PIDS.m_processorRotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(ProcessorRotations, SparkBase.ControlType.kPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double current = RobotContainer.m_elevatorSubsystem.getCurrentValue();
    boolean stopMotors = false;

    if(current >= Constants.ElevatorConstants.MAX_CURRENT){
      stopMotors = true;
    }
  
    return stopMotors;
    //return RobotContainer.m_elevatorSubsystem.CurrentSensingStop();
  }
}

