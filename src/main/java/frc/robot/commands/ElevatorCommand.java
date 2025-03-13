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

    //L1 Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_xButton)) {
      double L1PValue = Constants.ElevatorConstants.m_elevatorP;
      double L1IValue = Constants.ElevatorConstants.m_elevatorI;
      double L1DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L1PValue, L1IValue, L1DValue, 
      Constants.ElevatorConstants.L1_PIDS.m_L1MinOutput, Constants.ElevatorConstants.L1_PIDS.m_L1MaxOutput);
      double L1Rotations = Constants.ElevatorConstants.L1_PIDS.m_L1Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L1Rotations, SparkBase.ControlType.kPosition);
    }
    //L2 Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_bButton)) {
      double L2PValue = Constants.ElevatorConstants.m_elevatorP;
      double L2IValue = Constants.ElevatorConstants.m_elevatorI;
      double L2DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L2PValue, L2IValue, L2DValue, 
      Constants.ElevatorConstants.L2_PIDS.m_L2MinOutput, Constants.ElevatorConstants.L2_PIDS.m_L2MaxOutput);
      double L2Rotations = Constants.ElevatorConstants.L2_PIDS.m_L2Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L2Rotations, SparkBase.ControlType.kPosition);
    }
    //L3 Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_yButton)) {
      double L3PValue = Constants.ElevatorConstants.m_elevatorP;
      double L3IValue = Constants.ElevatorConstants.m_elevatorI;
      double L3DValue = Constants.ElevatorConstants.m_elevatorD;
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
    //Coral Station Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_aButton)) {
      double StationPValue = Constants.ElevatorConstants.m_elevatorP;
      double StationIValue = Constants.ElevatorConstants.m_elevatorI;
      double StationDValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(StationPValue, StationIValue, StationDValue, 
      Constants.ElevatorConstants.Station_PIDS.m_stationMinOutput, Constants.ElevatorConstants.Station_PIDS.m_stationMaxOutput);
      double ProcessorRotations = Constants.ElevatorConstants.Processor_PIDS.m_processorRotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(ProcessorRotations, SparkBase.ControlType.kPosition);
    }

    //"shift key" testing by mr ferguson 2025.03.08
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_xButton * Constants.OperatorConstants.JoystickButtons.m_leftStickButton)) {
      double A1PValue = Constants.ElevatorConstants.m_elevatorP;
      double A1IValue = Constants.ElevatorConstants.m_elevatorI;
      double A1DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(A1PValue, A1IValue, A1DValue, 
      Constants.ElevatorConstants.A1_PIDS.m_A1MinOutput, Constants.ElevatorConstants.A1_PIDS.m_A1MaxOutput);
      double A1Rotations = Constants.ElevatorConstants.A1_PIDS.m_A1Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(A1Rotations, SparkBase.ControlType.kPosition);
      System.out.println("test"); //Constants.OperatorConstants.JoystickButtons.m_xButton);
    }
    //end testing by mr ferguson
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

