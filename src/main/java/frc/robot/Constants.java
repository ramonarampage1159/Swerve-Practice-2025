// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final double maximumSpeed = Units.feetToMeters(11); //14.5 during competition

  public static class DriverConstants {

    public static final int m_driverController = 0;

    public static final double translationDEADBAND = 0.05;    
    public static final double rotationDEADBAND = 0.1;    

    public final class JoystickButtons{

      public static final int m_xButton = 1;
      public static final int m_aButton = 2;
      public static final int m_bButton = 3;

    }
    
  }

  public static class OperatorConstants {

    public static final int m_operatorController = 1;

    public final class JoystickButtons{

      public static final int m_xButton = 1;
      public static final int m_aButton = 2;
      public static final int m_bButton = 3;
      public static final int m_yButton = 4; 
      public static final int m_leftBumper = 5; 
      public static final int m_rightBumper = 6;
      public static final int m_leftTrigger = 7;
      public static final int m_rightTrigger = 8; 
      public static final int m_backButton = 9;
      public static final int m_startButton = 10;
      public static final int m_leftStickButton = 11;
      public static final int m_rightStickButton = 12;

    }

  }

  public static final class ElevatorConstants{

    public static final int m_leftMotor = 16;
    public static final int m_rightMotor = 15;

    public static final int MAX_CURRENT = 40;


    public final class ZeroPIDS{
      public static final double m_ZerokP = 0; 
      public static final double m_ZerokI = 0.000;
      public static final double m_ZerokD = 0.000; 
      public static final double m_ZerokIz = 0; 
      public static final double m_ZerokFF = 0; 
      public static final double m_ZerokMaxOutput = 0; 
      public static final double m_ZerokMinOutput = -0;

      public static final double m_ZerokRotations = 0;
    }
  
    public static class L1_PIDS{
      public static final double m_L1P = 0.05; 
      public static final double m_L1I = 0.00;
      public static final double m_L1D = 0.00; 
      public static final double m_L1Rotations = 27; 
      public static final double m_L1MaxOutput = 0.5; 
      public static final double m_L1MinOutput = -0.5;
    }

    public static class L2_PIDS{
      public static final double m_L2P = 0.05; 
      public static final double m_L2I = 0.00;
      public static final double m_L2D = 0.00; 
      public static final double m_L2Rotations = 54; 
      public static final double m_L2MaxOutput = 0.5; 
      public static final double m_L2MinOutput = -0.5;
    }

    public static class L3_PIDS{
      public static final double m_L3P = 0.05; 
      public static final double m_L3I = 0.00;
      public static final double m_L3D = 0.00; 
      public static final double m_L3Rotations = 81; 
      public static final double m_L3MaxOutput = 0.5; 
      public static final double m_L3MinOutput = -0.5;
    }

    public static class Processor_PIDS{
      public static final double m_processorP = 0.05; 
      public static final double m_processorI = 0.00;
      public static final double m_processorD = 0.00; 
      public static final double m_processorRotations = 0; 
      public static final double m_processorMaxOutput = 0.5; 
      public static final double m_processorMinOutput = -0.5;
    }

  }

  public static final class CoralConstants{
    public static final int m_motorCoralRight = 17;
    public static final int m_motorCoralLeft = 18;
  }

  public static class AutoConstants {
      public static final double CoralShootTimeLimitSeconds = 2;
    

  }
  

}


