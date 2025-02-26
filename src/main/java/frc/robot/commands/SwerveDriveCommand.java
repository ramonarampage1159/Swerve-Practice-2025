package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveCommand extends Command{
    public static boolean m_FieldCentricMode = true;
    private final SwerveDriveSubsystem m_subsystem;



public SwerveDriveCommand(SwerveDriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = driveSubsystem;
    addRequirements(m_subsystem);
    }


   // Called when the command is initially scheduled.
   @Override
   public void initialize() {}
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
   
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
