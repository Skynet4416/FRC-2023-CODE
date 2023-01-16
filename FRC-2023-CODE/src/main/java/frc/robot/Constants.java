// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.drive.DrivingState;

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
  public static class OI{
    public static final int kLeftJoystickControllerPort = 1;
    public static final int kRightJoystickControllerPort = 2;
    public static final int kXboxControllerPort = 0;
    public static final double kXboxcontrollerDrift = 0.1;
  }

  public static class Drive {
    public static final DrivingState kDriveState = DrivingState.CURVATURE;

    public static class Motors {
      public static final MotorType kMotorType = MotorType.kBrushless;
      public static final int kLeftForwardCANID = 10;
      public static final int kRightForwardCANID = 11;
      public static final int kLeftBackwardCANID = 12;
      public static final int kRightBackwardCANID = 13;
    }

    public static class CheatedEncodersPorts {
      public static final int[] kRightEncoderPorts = { 2, 3 };
      public static final int[] kLeftEncoderPorts = { 4, 5 };
      public static final boolean kLeftEncoderReversed = false;
      public static final boolean kRightEncoderReversed = true;
      public static final int kEncoderCPR = 1024;
      public static final double kEncoderDistancePerPulse = (Physical.kwheelDiamaterInMeters * Math.PI) / (double) kEncoderCPR;
    }

    public static class PID {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kS = 0;
      public static final double kvVoltSecondsPerMeter = 1.98;
      public static final double kaVoltSecondsSquaredPerMeter = 1.2;
    }

    public static class PIDAngular {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kS = 0;
      public static final double kvVoltSecondsPerRadian = 1.5;
      public static final double kaVoltSecondsSquaredPerRadian = 0.3;
    }

    public static class Physical {
      public static final double kwheelDiamaterInMeters = Units.inchesToMeters(6);
      public static final double kwheelRadiusInMeters = kwheelDiamaterInMeters / 2;
      public static final double kGearReduaction = 10.71;
      public static final double kGearRatio = 1 / kGearReduaction;
      public static final double kUnitsToMeters = kGearRatio * Math.pow(kwheelRadiusInMeters, 2) * Math.PI;
      public static final double kUnitsPerMinuteToMeterPerSecond = kGearRatio * Math.pow(kwheelRadiusInMeters, 2)
          * Math.PI / 60;
      public static final double kDistanceBetweenLeftAndRightWheelsInMeters = Units.inchesToMeters(28);
      public static final Vector<N3> kStateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
      public static final Vector<N3> kVisionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
      public static final Pose2d kDeafultPosition = new Pose2d();
      // ------------------ ERROR HERE ------------------
      public static final LinearSystem<N2, N2, N2> kDriveTrainPlant = LinearSystemId.identifyDrivetrainSystem(
          PID.kvVoltSecondsPerMeter,
          PID.kaVoltSecondsSquaredPerMeter,
          PIDAngular.kvVoltSecondsPerRadian,
          PIDAngular.kaVoltSecondsSquaredPerRadian);
      public static final Vector<N7> kMessurmentStdDevs = VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005);
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
