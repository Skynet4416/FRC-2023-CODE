// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.data.Matrix;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
  public static class Drive {
    public static class Motors {
      public static final MotorType kMotorType = MotorType.kBrushless;
      public static final int kLeftForwardCANID = 10;
      public static final int kRightForwardCANID = 11;
      public static final int kLeftBackwardCANID = 12;
      public static final int kRightBackwardCANID = 13;
    }

    public static class PID {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kS = 0;
      public static final double kvVoltSecondsPerMeter = 0;
      public static final double kaVoltSecondsSquaredPerMeter = 0;
    }

    public static class PIDAngular {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kS = 0;
      public static final double kvVoltSecondsPerRadian = 0;
      public static final double kaVoltSecondsSquaredPerRadian = 0;
    }

    public static class Physical{
      public static final double kwheelDiamaterInMeters = Units.inchesToMeters(6);
      public static final double kwheelRadiusInMeters = kwheelDiamaterInMeters/2;
      public static final double kGearReduaction = 10.71;
      public static final double kGearRatio = 1/kGearReduaction;
      public static final double kUnitsToMeters = kGearRatio * Math.pow(kwheelRadiusInMeters,2) * Math.PI;
      public static final double kUnitsPerMinuteToMeterPerSecond = kGearRatio * Math.pow(kwheelRadiusInMeters,2) * Math.PI/60;
      public static final double kDistanceBetweenLeftAndRightWheelsInMeters = Units.inchesToMeters(28);
      public static final Vector<N3> kStateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
      public static final Vector<N3> kVisionMeasurementStdDevs =  VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
      public static final Pose2d kDeafultPosition = new Pose2d();
      public static final LinearSystem kDriveTrainPlant=  LinearSystemId.identifyDrivetrainSystem(
        PID.kvVoltSecondsPerMeter,
        PID.kaVoltSecondsSquaredPerMeter,
        PIDAngular.kvVoltSecondsPerRadian,
        PIDAngular.kaVoltSecondsSquaredPerRadian);
        
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
