package frc.robot.commands.AutoCommands;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class CoralShootAuto extends Command{
     /** Creates a new CoralShootAuto. */

   Timer m_timer = new Timer();
   double m_time;


   public CoralShootAuto(double time) {
    m_time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_coralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_coralSubsystem.shootCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_time)){
      RobotContainer.m_coralSubsystem.stopMotors();
      return true;
    }
    else{
    return false;
    }
  }
    
}


