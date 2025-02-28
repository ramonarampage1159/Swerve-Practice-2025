package frc.robot.commands.AutoCommands;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class L1CoralAuto extends Command{
    
   /** Creates a new L1CoralAuto. */
   public L1CoralAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double L1PValue = Constants.ElevatorConstants.L1_PIDS.m_L1P;
      double L1IValue = Constants.ElevatorConstants.L1_PIDS.m_L1I;
      double L1DValue = Constants.ElevatorConstants.L1_PIDS.m_L1D;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L1PValue, L1IValue, L1DValue, 
      Constants.ElevatorConstants.L1_PIDS.m_L1MinOutput, Constants.ElevatorConstants.L1_PIDS.m_L1MaxOutput);
      double L1Rotations = Constants.ElevatorConstants.L1_PIDS.m_L1Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L1Rotations, SparkBase.ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}