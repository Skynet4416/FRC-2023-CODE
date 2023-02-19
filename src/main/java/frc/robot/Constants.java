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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.drive.DrivingState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Map;
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
public class Constants {

  /**
   * Contains various field dimensions and useful reference points. Dimensions are
   * in meters, and sets
   * of corners start in the lower left moving clockwise.
   *
   * <p>
   * All translations and poses are stored with the origin at the rightmost point
   * on the BLUE
   * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and
   * {@link #allianceFlip(Pose2d)}
   * methods to flip these values based on the current alliance color.
   */
  public static class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);

    // Dimensions for community and charging station, including the tape.
    public static class Community {
      // Region dimensions
      public static final double innerX = 0.0;
      public static final double midX = Units.inchesToMeters(132.375); // Tape to the left of charging station
      public static final double outerX = Units.inchesToMeters(193.25); // Tape to the right of charging station
      public static final double leftY = Units.feetToMeters(18.0);
      public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
      public static final double rightY = 0.0;
      public static final Translation2d[] regionCorners = new Translation2d[] {
          new Translation2d(innerX, rightY),
          new Translation2d(innerX, leftY),
          new Translation2d(midX, leftY),
          new Translation2d(midX, midY),
          new Translation2d(outerX, midY),
          new Translation2d(outerX, rightY),
      };

      // Charging station dimensions
      public static final double chargingStationLength = Units.inchesToMeters(76.125);
      public static final double chargingStationWidth = Units.inchesToMeters(97.25);
      public static final double chargingStationOuterX = outerX - tapeWidth;
      public static final double chargingStationInnerX = chargingStationOuterX - chargingStationLength;
      public static final double chargingStationLeftY = midY - tapeWidth;
      public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
      public static final Translation2d[] chargingStationCorners = new Translation2d[] {
          new Translation2d(chargingStationInnerX, chargingStationRightY),
          new Translation2d(chargingStationInnerX, chargingStationLeftY),
          new Translation2d(chargingStationOuterX, chargingStationRightY),
          new Translation2d(chargingStationOuterX, chargingStationLeftY)
      };

      // Cable bump
      public static final double cableBumpInnerX = innerX + Grids.outerX + Units.inchesToMeters(95.25);
      public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
      public static final Translation2d[] cableBumpCorners = new Translation2d[] {
          new Translation2d(cableBumpInnerX, 0.0),
          new Translation2d(cableBumpInnerX, chargingStationRightY),
          new Translation2d(cableBumpOuterX, 0.0),
          new Translation2d(cableBumpOuterX, chargingStationRightY)
      };
    }

    // Dimensions for grids and nodes
    public static class Grids {
      // X layout
      public static final double outerX = Units.inchesToMeters(54.25);
      public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
      public static final double midX = outerX - Units.inchesToMeters(22.75);
      public static final double highX = outerX - Units.inchesToMeters(39.75);

      // Y layout
      public static final int nodeRowCount = 9;
      public static final double nodeFirstY = Units.inchesToMeters(20.19);
      public static final double nodeSeparationY = Units.inchesToMeters(22.0);

      // Z layout
      public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
      public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
      public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
      public static final double highConeZ = Units.inchesToMeters(46.0);
      public static final double midConeZ = Units.inchesToMeters(34.0);

      // Translations (all nodes in the same column/row have the same X/Y coordinate)
      public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
      public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
      public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
      public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
      public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

      static {
        for (int i = 0; i < nodeRowCount; i++) {
          boolean isCube = i == 1 || i == 4 || i == 7;
          lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
          midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
          mid3dTranslations[i] = new Translation3d(midX, nodeFirstY + nodeSeparationY * i,
              isCube ? midCubeZ : midConeZ);
          high3dTranslations[i] = new Translation3d(
              highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
          highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
        }
      }

      // Complex low layout (shifted to account for cube vs cone rows and wide edge
      // nodes)
      public static final double complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone
                                                                                               // nodes
      public static final double complexLowXCubes = lowX; // Centered X under cube nodes
      public static final double complexLowOuterYOffset = nodeFirstY - Units.inchesToMeters(3.0)
          - (Units.inchesToMeters(25.75) / 2.0);

      public static final Translation2d[] complexLowTranslations = new Translation2d[] {
          new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
          new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
          new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
          new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
          new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
          new Translation2d(
              complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),
      };
    }

    // Dimensions for loading zone and substations, including the tape
    public static class LoadingZone {
      // Region dimensions
      public static final double width = Units.inchesToMeters(99.0);
      public static final double innerX = FieldConstants.fieldLength;
      public static final double midX = fieldLength - Units.inchesToMeters(132.25);
      public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
      public static final double leftY = FieldConstants.fieldWidth;
      public static final double midY = leftY - Units.inchesToMeters(50.5);
      public static final double rightY = leftY - width;
      public static final Translation2d[] regionCorners = new Translation2d[] {
          new Translation2d(
              midX, rightY), // Start at lower left next to border with opponent community
          new Translation2d(midX, midY),
          new Translation2d(outerX, midY),
          new Translation2d(outerX, leftY),
          new Translation2d(innerX, leftY),
          new Translation2d(innerX, rightY),
      };

      // Double substation dimensions
      public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
      public static final double doubleSubstationX = innerX - doubleSubstationLength;
      public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);

      // Single substation dimensions
      public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
      public static final double singleSubstationLeftX = FieldConstants.fieldLength - doubleSubstationLength
          - Units.inchesToMeters(88.77);
      public static final double singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0);
      public static final double singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
      public static final Translation2d singleSubstationTranslation = new Translation2d(singleSubstationCenterX, leftY);

      public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
      public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
      public static final double singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0);
      public static final double singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;
    }

    // Locations of staged game pieces
    public static class StagingLocations {
      public static final double centerOffsetX = Units.inchesToMeters(47.36);
      public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
      public static final double firstY = Units.inchesToMeters(36.19);
      public static final double separationY = Units.inchesToMeters(48.0);
      public static final Translation2d[] translations = new Translation2d[4];

      static {
        for (int i = 0; i < translations.length; i++) {
          translations[i] = new Translation2d(positionX, firstY + (i * separationY));
        }
      }
    }

    // AprilTag locations (do not flip for red alliance)
    public static final Map<Integer, Pose3d> aprilTags = Map.of(
        1,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        2,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        3,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        4,
        new Pose3d(
            Units.inchesToMeters(636.96),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d(0.0, 0.0, Math.PI)),
        5,
        new Pose3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d()),
        6,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        7,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        8,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()));
  }

  public static class OI {
    public static final int kLeftJoystickControllerPort = 1;
    public static final int kRightJoystickControllerPort = 2;
    public static final int kXboxControllerPort = 0;
    public static final double kXboxcontrollerDrift = 0.1;
  }

  public static class Vision {

    public static class AprilTag {
      public static final String kCameraName = "AprilTag Camera";
    }

    public static class ReflectiveTape {
      public static final String kReflectiveTapeCameraName = "Reflective Tape Camera";
    }
  }

  public static class Drive {
    public static final DrivingState kDriveState = DrivingState.CURVATURE;
    public static final boolean kKalmanPoseEstimation = false;

    public static class Trajectory {
      public static final double kMaxVelocity = 4.0;
      public static final double kMaxAcceleration = 4.0;
      public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(kMaxVelocity, kMaxAcceleration);
    }

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
      public static final double kEncoderDistancePerPulse = (Physical.kwheelDiamaterInMeters * Math.PI)
          / (double) kEncoderCPR;
    }

    public static class PID {
      public static final double kP = 8.5;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kSVolts = 0.22;
      public static final double kVVoltSecondsPerMeter = 1.98;
      public static final double kAVoltSecondsSquaredPerMeter = 1.2;
    }

    public static class PIDAngular {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kSVolts = 0;
      public static final double kvVoltSecondsPerRadian = 1.5;
      public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    }
    public static class ChargeStationPID{
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;

    }

    public static class RamseteController {
      public static final double kB = 2;
      public static final double kZeta = 0.7;
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
      public static final LinearSystem<N2, N2, N2> kDriveTrainPlant = LinearSystemId.identifyDrivetrainSystem(
          PID.kVVoltSecondsPerMeter,
          PID.kAVoltSecondsSquaredPerMeter,
          PIDAngular.kvVoltSecondsPerRadian,
          PIDAngular.kaVoltSecondsSquaredPerRadian);
      public static final Vector<N7> kMessurmentStdDevs = VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005);
      public static final double kMaxVelcoityMeterPerSecond = 4;
      public static final double kMaxAccelerationMeterPerSecondSquered = 4;
      public static final double kMaxRotationalVelocityRadiansPerSecond = 4;
      public static final double kMaxRotationalAccelerationRadiansPerSecondSquered = 4;
    }
  }

  public static class Intake {
    public static class Motors {
      public static final MotorType kMotorType = MotorType.kBrushless;
      public static final int kLeftIntakeCANID = 14;
      public static final int kRightIntakeCANID = 15;
    }
  }

  public static class Wrist {
    public static class Motors {
      public static final int kWristCANID = -1;
    }

    public static class Physical {
      public static final double kMaxVelcoityRadiansPerSecond = 0.0;
      public static final double kMaxAccelerationRadiansPerSecondSquered = 0.0;

    }

    public static class PID {
      public final static double kP = 0;
      public final static double kD = 0;
      public final static double kI = 0;

    }

    public static class FeedForward {
      public final static double kS = 0;
      public final static double kG = 0;
      public final static double kV = 0;
      public final static double kA = 0;

    }

    public static class Encoders {
      public static final int kWristCANID = -1;
      public static final double kCANCoderZeroAbsAngle = 0;
    }

  }

  public static class Arm {
    public static class Motors {
      public static final MotorType kMotorType = MotorType.kBrushless;
      public static final int kArmCANID = 16;
      public static final double falconUnitsPerRotation = 4096.0;
    }

    public static class MotionMagicPID {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0;
      public static final double kG = 0;
    }

    public static class PoistionPID {
      public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }

      public static final class ArbitraryFeedForward {
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

      }
    }

    public static class Physical {
      public static double kHeightThreasholdInMeters = 0.01;
      public static double kArmMininunAngleInRadians = Units.degreesToRadians(80);
      public static double kArmMaximumAngleInRadians = Units.degreesToRadians(180);
      public static double kArmMass = 6;
      public static double kArmMaxVelocityRadiansPerSecond = Units.degreesToRadians(120);
      public static double kArmMaxAccelerationRadiansPerSecondSquered = Units.degreesToRadians(20);
      public static final double kArmLength = 1;
      // TODO: Measure the actual value (THIS IS NOT CORRECT)
      public static final double kFloorHeightInMeters = 2.0;
      public static final double kMiddleHeightInMeters = 5.0;
      public static final double kTopHeightInMeters = 8.0;
      public static final double kArmMomentOfInertia = kArmMass * Math.pow(0.62, 2); // kg * m^2
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
