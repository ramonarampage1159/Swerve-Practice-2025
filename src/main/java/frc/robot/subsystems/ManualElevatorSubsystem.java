package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex; 
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;


public class ManualElevatorSubsystem extends SubsystemBase {
    
  private final SparkFlex m_motorLeft = new SparkFlex(Constants.ElevatorConstants.m_leftMotor, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex m_motorRight = new SparkFlex(Constants.ElevatorConstants.m_rightMotor, SparkLowLevel.MotorType.kBrushless);

  private SparkFlexConfig m_motorRightConfig = new SparkFlexConfig();
    

  public ManualElevatorSubsystem() {

    m_motorRightConfig.follow(m_motorLeft, true);

    m_motorRight.configure(m_motorRightConfig,null,null);

  }

  public void ElevatorUp() {
    m_motorLeft.set(0.2); 
  }

  public void ElevatorDown() {
    m_motorLeft.set(-0.2); 
  }

  public void stopMotors() {
    m_motorLeft.set(0);
  }

  public double getCurrentValue(){
    double current = m_motorLeft.getOutputCurrent();
    return current;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}