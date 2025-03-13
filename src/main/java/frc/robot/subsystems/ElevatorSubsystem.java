package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase{
    /** Creates a new ArmSubsystem. */
    private SparkFlex m_leftElevatorMotor = new SparkFlex(Constants.ElevatorConstants.m_leftMotor, SparkLowLevel.MotorType.kBrushless);
    private SparkFlex m_rightElevatorMotor = new SparkFlex(Constants.ElevatorConstants.m_rightMotor, SparkLowLevel.MotorType.kBrushless);
    
    private SparkClosedLoopController m_LeftClosedLoopController;

    private SparkFlexConfig m_motorLeftConfig = new SparkFlexConfig(); 
    private SparkFlexConfig m_motorRightConfig = new SparkFlexConfig();
    
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    public double rotations;

    public double current = m_leftElevatorMotor.getOutputCurrent();


  public ElevatorSubsystem() {

    m_motorRightConfig.follow(m_leftElevatorMotor, true);
    m_LeftClosedLoopController = m_leftElevatorMotor.getClosedLoopController();

    
    kP = Constants.ElevatorConstants.ZeroPIDS.m_ZerokP; 
    kI = Constants.ElevatorConstants.ZeroPIDS.m_ZerokI;
    kD = Constants.ElevatorConstants.ZeroPIDS.m_ZerokD; 
    kIz = Constants.ElevatorConstants.ZeroPIDS.m_ZerokIz; 
    kFF = Constants.ElevatorConstants.ZeroPIDS.m_ZerokFF; 
    kMaxOutput = Constants.ElevatorConstants.ZeroPIDS.m_ZerokMaxOutput; 
    kMinOutput = Constants.ElevatorConstants.ZeroPIDS.m_ZerokMinOutput;

    //setAllPIDValues(kP,kI,kD,kIz,kFF,kMinOutput,kMaxOutput);

    m_motorLeftConfig.closedLoop.pid(kP,kI, kD);

    m_rightElevatorMotor.configure(m_motorRightConfig,null,null);
    m_leftElevatorMotor.configure(m_motorLeftConfig,null,null);


  }

  public void setArmReference(double speed, ControlType type) {

    this.rotations = speed;
    m_LeftClosedLoopController.setReference(speed, type);

  }

  public boolean isAtSetpoint() {
    return Math.abs(m_leftElevatorMotor.getEncoder().getPosition() - rotations) <= 0.5;
  }

  public void setAllPIDValues(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput){

    m_motorLeftConfig.closedLoop.pidf(kP,kI, kD, kFF);
    m_motorLeftConfig.closedLoop.iZone(kIz);
    m_motorLeftConfig.closedLoop.minOutput(kMinOutput);
    m_motorLeftConfig.closedLoop.maxOutput(kMaxOutput);

    m_leftElevatorMotor.configure(m_motorLeftConfig,null,null);
    
  }


  public void setPIDValues(double kP, double kI, double kD, double kMinOutput, double kMaxOutput) {

    m_motorLeftConfig.closedLoop.pid(kP,kI, kD);
    m_motorLeftConfig.closedLoop.minOutput(kMinOutput);
    m_motorLeftConfig.closedLoop.maxOutput(kMaxOutput);

    m_leftElevatorMotor.configure(m_motorLeftConfig,null,null);
  }

  //THIS OR TAHT
  public double getCurrentValue(){
    return current;
  }

  /*
  //THIS OR THAT
  public void CurrentSensing(){
    double current = m_leftElevatorMotor.getOutputCurrent();
    boolean stopMotors = false;

    if(current >= Constants.ElevatorConstants.MAX_CURRENT){
      stopMotors = true;
      SmartDashboard.putBoolean("Current Sensing Elevator Stop", stopMotors);
    }
  }
  */
  public boolean CurrentSensingStop(){
    boolean stopMotors = false;

    if(current >= Constants.ElevatorConstants.MAX_CURRENT){
      stopMotors = true;
      SmartDashboard.putBoolean("Current Sensing Elevator Stop", stopMotors);
    }
    return stopMotors;
  }


 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    SmartDashboard.putNumber("arm right",m_rightElevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("arm left",m_leftElevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("arm left current",m_leftElevatorMotor.getOutputCurrent());
    SmartDashboard.putBoolean("arm at setpoint", isAtSetpoint());
    */
    SmartDashboard.putNumber("Elevator Position", m_leftElevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Current TEST", getCurrentValue());

    //CurrentSensingStop();

  }
}