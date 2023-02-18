// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

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
    public static class MotionMagicPID{
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0;
      public static final double kG = 0;
    }
    public static class PoistionPID{
      public static final class PID{
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }
      public static final class ArbitraryFeedForward{
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

      }
    }

    public static class Physical{
      public static double kHeightThreasholdInMeters=0.01;
      public static double kArmMininunAngleInRadians=Units.degreesToRadians(80);
    public static double kArmMaximumAngleInRadians = Units.degreesToRadians(180);
    public static double kArmMass = 6;
      public static final double kArmLength = 2;
      //TODO: Measure the actual value (THIS IS NOT CORRECT)
      public static final double kFloorHeightInMeters = 2.0;
      public static final double kMiddleHeightInMeters = 5.0;
      public static final double kTopHeightInMeters = 8.0;
      public static final double kArmMomentOfInertia = 1.91800 ; //kg * m^2
      public static final double kMotorGearing = 100;
      public static final double kExtraGearing = 1;
      public static final double kArmGearing = kMotorGearing * kExtraGearing;
      public static final double kArmHeight = 2;
      public static final double kIntakeGroundHeight = 0;
      public static final double kIntakeDoubleSubstationHeight = 1.2;
      public static final double kGroundGridHeight = 0.2;
      public static final double kCubeMidGridHeight = 0.6;
      public static final double kConeMidGridHeight = 0.6;
      public static final double kCubeHighGridHeight = 1.2;
      public static final double kConeHighGridHeight = 0;
    }

    public static class Encoders {
      public static final int kCANCoderID = 17;
      public static final double kCANCoderZeroAngle = 0;
      public static final double kTimeDelayOfSensor = 0.025;
    }
  }
}
