// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Drive{
    public static class Motors{
      public static MotorType kMotorType = MotorType.kBrushless;
      public static int kLeftForwardCANID = 10;
      public static int kRightForwardCANID = 11;
      public static int kLeftBackwardCANID = 12;
      public static int kRightBackwardCANID = 13;
    }
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
    public static class OI {
      public static final int kLeftJoystickControllerPort = 1;
      public static final int kRightJoystickControllerPort = 2;
      public static final int kXboxControllerPort = 0;
      public static final double kXboxcontrollerDrift = 0.1;
  }

  public static class Arm {
    public static class Motors {
      public static MotorType kMotorType = MotorType.kBrushless;
      public static int kArmCANID = 16;
      public static double falconUnitsPerRotation = 4096.0;
    }
    public static class Physical{
      public static double kHeightThreasholdInMeters=0.01; 
    }

    public static class Encoders {
      public static int kCANCoderID = 17;
    }
  }
}
