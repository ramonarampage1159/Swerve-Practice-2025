package frc.robot.commands.AutoCommands;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class L2CoralAuto extends Command{
    
   /** Creates a new L2CoralAuto. */
   public L2CoralAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double L2PValue = Constants.ElevatorConstants.m_elevatorP;
      double L2IValue = Constants.ElevatorConstants.m_elevatorI;
      double L2DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L2PValue, L2IValue, L2DValue, 
      Constants.ElevatorConstants.L2_PIDS.m_L2MinOutput, Constants.ElevatorConstants.L2_PIDS.m_L2MaxOutput);
      double L2Rotations = Constants.ElevatorConstants.L2_PIDS.m_L2Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L2Rotations, SparkBase.ControlType.kPosition);
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