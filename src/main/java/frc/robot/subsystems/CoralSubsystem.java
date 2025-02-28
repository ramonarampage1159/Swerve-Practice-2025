package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel;

import frc.robot.Constants;


public class CoralSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    private SparkMax m_motorCoralRight, m_motorCoralLeft;
    private SparkMaxConfig m_motorCoralRightConfig,m_motorCoralLeftConfig;

    public CoralSubsystem() {
        m_motorCoralRight = new SparkMax(Constants.CoralConstants.m_motorCoralRight, SparkLowLevel.MotorType.kBrushless);
        m_motorCoralLeft = new SparkMax(Constants.CoralConstants.m_motorCoralLeft, SparkLowLevel.MotorType.kBrushless);

        m_motorCoralRightConfig = new SparkMaxConfig();
        m_motorCoralLeftConfig = new SparkMaxConfig();

        m_motorCoralRightConfig.follow(m_motorCoralLeft, true);

        m_motorCoralRight.configure(m_motorCoralRightConfig,null,null);
        m_motorCoralLeft.configure(m_motorCoralLeftConfig,null,null);
    }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public void IntakeCoral() {
    m_motorCoralLeft.set(0.2);
  }


  public void stopMotors(){
    m_motorCoralLeft.set(0);
  }


  public void shootCoral(){
    m_motorCoralLeft.set(-0.2);
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
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
