// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
      public static final MotorType kMotorType = MotorType.kBrushless;
      public static final int kArmCANID = 16;
      public static final double falconUnitsPerRotation = 4096.0;
    }

    public static class Physical{
      public static double kHeightThreasholdInMeters=0.01;

      //TODO: Measure the actual value (THIS IS NOT CORRECT)
      public static final double kFloorHeightInMeters = 2.0;
      public static final double kMiddleHeightInMeters = 5.0;
      public static final double kTopHeightInMeters = 8.0;
      public static final double kArmMomentOfInertia = 1.2; //kg * m^2
      public static final double kArmGearing = 10;
    }

    public static class Encoders {
      public static final int kCANCoderID = 17;
    }
  }
}
