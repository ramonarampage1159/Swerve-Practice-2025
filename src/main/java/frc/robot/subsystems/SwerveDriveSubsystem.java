// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkFlex;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;



public class SwerveDriveSubsystem extends SubsystemBase {

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;

  AHRS m_gyro;

  /*
  //TEST FOR SMARTDASHBOARD
  private DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(0);
  private Encoder m_relativeEncoder = new Encoder(1, 2);
   */



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
    
    m_gyro = (AHRS)swerveDrive.getGyro().getIMU();

   setupPathPlanner();

    //set angle motors to brake mode
    new Thread(()->{
      try{
        Thread.sleep(1000);
        //angleMotorBrake();
      } catch (Exception e){
      }
      }).start();
      System.out.println("angle motors set brake mode");
   
  }


  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }


  public void angleMotorBrake(){

    for(SwerveModule module : swerveDrive.getModules()){
      module.getAngleMotor().setMotorBrake(true);
      module.getAngleMotor().burnFlash();    
    }

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


  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get()) == DriverStation.Alliance.Red)
      {
        swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
      }else
      {
        swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble() *-1, 3) * swerveDrive.getMaximumChassisVelocity(),
                                          Math.pow(translationY.getAsDouble() *-1, 3) * swerveDrive.getMaximumChassisVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
      }
    });
  }


 /* 
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
*/

  public Command driveRobotCentricCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);

    });
  }

  private void updateSmartDashboard() {

    //SmartDashboard.putNumber("Yaw Pub", swerveDrive.getYaw().getDegrees());

    double angle = m_gyro.getYaw();
    SmartDashboard.putNumber("Yaw Pub", angle);


    for(SwerveModule m : swerveDrive.getModules())
    {
      CANcoder absoluteEncoder = (CANcoder)m.configuration.absoluteEncoder.getAbsoluteEncoder();

      String AbsEncoderID = Integer.toString(absoluteEncoder.getDeviceID());
    
      double AbsoluteEncoderDeg = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
      AbsoluteEncoderDeg *= 360.0;

      double InternalAngle = AbsoluteEncoderDeg - m.configuration.angleOffset;

      double ModuleSpeed = absoluteEncoder.getVelocity().getValueAsDouble();

      SmartDashboard.putNumber("Raw Abs Encoder #" + AbsEncoderID, AbsoluteEncoderDeg);   
      SmartDashboard.putNumber("Internal Angle Encoder #" + AbsEncoderID, InternalAngle);
      SmartDashboard.putNumber("Module " + AbsEncoderID + " Speed", ModuleSpeed);

    }


    for(SwerveModule module : swerveDrive.getModules())
    {
        SparkFlex driveMotor = (SparkFlex)module.configuration.driveMotor.getMotor();

        String driveMotorID = Integer.toString(driveMotor.getDeviceId());

        double driveMotorVelocity = driveMotor.getEncoder().getVelocity();
          
      SmartDashboard.putNumber("Drive Motor # " + driveMotorID, driveMotorVelocity);   

    }

    /*
    //TEST for SmartDashboard
    int count = m_relativeEncoder.get()/2048;
    double RelativeDistance = m_relativeEncoder.get()/2048.000*5.00000000*Math.PI;
    
    SmartDashboard.putNumber("RelativeDistance", RelativeDistance);   
    SmartDashboard.putNumber("Count", count);  
     */

    
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

  public void setMotorSpeed(double xAxis, double yAxis, double  zAxis) {
    
  }


   /**
   * Setup AutoBuilder for PathPlanner.
   */

  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              /* 02/17 part of original working (kind of) code
              new PIDConstants(5.0, 0.0, 0.0), */
              new PIDConstants(0.02, 0.0, 0.0),
              // Translation PID constants
              /* 02/17 part of original working (kind of) code
              new PIDConstants(5.0, 0.0, 0.0) */
              new PIDConstants(0.05, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

   /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

}
