package frc.robot.commands.AutoCommands;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class L3CoralAuto extends Command{
    
   /** Creates a new L3CoralAuto. */
   public L3CoralAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double L3PValue = Constants.ElevatorConstants.m_elevatorP;
      double L3IValue = Constants.ElevatorConstants.m_elevatorI;
      double L3DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L3PValue, L3IValue, L3DValue, 
      Constants.ElevatorConstants.L3_PIDS.m_L3MinOutput, Constants.ElevatorConstants.L3_PIDS.m_L3MaxOutput);
      double L3Rotations = Constants.ElevatorConstants.L3_PIDS.m_L3Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L3Rotations, SparkBase.ControlType.kPosition);
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