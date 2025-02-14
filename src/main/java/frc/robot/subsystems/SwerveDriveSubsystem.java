// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ejml.equation.Variable;

import com.ctre.phoenix6.hardware.CANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
//import com.ctre.phoenix6.signals.AbsoluteSensorDiscontinuityPoint;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;


public class SwerveDriveSubsystem extends SubsystemBase {

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;
  //Rotation2d YawPub;


  AHRS navx;
  AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  /** Creates a new ExampleSubsystem. */
  public SwerveDriveSubsystem() {
    
    //change or delete line during competition
    //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.maximumSpeed,
                                                                    new Pose2d(new Translation2d(Meter.of(1),
                                                                                                 Meter.of(4)),
                                                                               Rotation2d.fromDegrees(0)));
    }
    catch (Exception e){
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.


    navx = (AHRS)swerveDrive.getGyro().getIMU();
  }
 //test

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

  /*public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
             Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }*/



  //new 02/12
  /*public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      //swerveDrive.getMaximumVelocity(),
                                                                      swerveDrive.getMaximumChassisVelocity()
                                                                      ));
    });
  }*/

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }
  




  

  /*
   * public class Robot extends TimedRobot
{

  private static Robot   instance;
  private CANcoder    absoluteEncoder;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }
   */

  

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  
  /**
  
   *    @Override
    public void robotInit()
  {
    absoluteEncoder = new CANcoder(Change this to the CAN ID of the CANcoder 0);
   CANcoderConfigurator cfg = encoder.getConfigurator();
   cfg.apply(new CANcoderConfiguration());
   MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
   cfg.refresh(magnetSensorConfiguration);
   cfg.apply(magnetSensorConfiguration
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
                  
  }
  */


  
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

   /*
    * @Override
  public void robotPeriodic()
  {
   StatusSignal<Double> angle = encoder.getAbsolutePosition().waitForUpdate(0.1);

   System.out.println("Absolute Encoder Angle (degrees): " + Units.rotationsToDegrees(angle.getValue()));
  }
}
  
    */
  
  private void updateSmartDashboard() {

    //SmartDashboard.putNumber("Yaw Pub", swerveDrive.getYaw().getDegrees());

    double angle = m_gyro.getAngle();
    double angle2 = navx.getAngle();
    SmartDashboard.putNumber("Yaw Pub", angle);
    SmartDashboard.putNumber("Yaw Pub 2", angle2);



    

    // Rotation2d.fromRadians(imuReadingCache.getValue().getZ());


    //Rotation2d.fromRadians()
   // return Rotation2d.fromRadians(imuReadingCache.getValue().getZ());



    for(SwerveModule m : swerveDrive.getModules())
    {
      //System.out.println("Module Name: "+m.configuration.name);
      CANcoder absoluteEncoder = (CANcoder)m.configuration.absoluteEncoder.getAbsoluteEncoder();

      String AbsEncoderID = Integer.toString(absoluteEncoder.getDeviceID());
    
      //output here

      double AbsoluteEncoderDeg = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
      AbsoluteEncoderDeg *= 360.0;

      SmartDashboard.putNumber("Raw Abs Encoder #" + AbsEncoderID, AbsoluteEncoderDeg);   
    }
    






    
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
    updateSmartDashboard();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      SmartDashboard.putNumber("Rotation Axis", 2);
      SmartDashboard.putNumber("Left to Right Axis", 1);
      
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }




}
