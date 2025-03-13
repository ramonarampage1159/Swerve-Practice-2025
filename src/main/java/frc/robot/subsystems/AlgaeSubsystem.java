// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */



  
  private SparkMax m_motorAlgaeRight, m_motorAlgaeLeft;

  public AlgaeSubsystem() {
    m_motorAlgaeRight = new SparkMax(Constants.AlgaeConstants.m_motorAlgaeRight, SparkLowLevel.MotorType.kBrushless);
    m_motorAlgaeLeft = new SparkMax(Constants.AlgaeConstants.m_motorAlgaeLeft, SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig m_motorAlgaeRightConfig = new SparkMaxConfig();
    SparkMaxConfig m_motorAlgaeLeftConfig = new SparkMaxConfig();

    m_motorAlgaeRightConfig.follow(m_motorAlgaeLeft, true); 

   m_motorAlgaeRight.configure(m_motorAlgaeRightConfig,null,null);
   m_motorAlgaeLeft.configure(m_motorAlgaeLeftConfig,null,null);

  
  
  }
  
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          //one-time action goes here 
        });
  }
  public void IntakeAlgae() { 
    m_motorAlgaeLeft.set(0.2);
  }

  public void stopMotors(){
    m_motorAlgaeLeft.set(0);
  }

  public void shootAlgae(){
    m_motorAlgaeLeft.set(-0.2);
  }

  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

   
}
