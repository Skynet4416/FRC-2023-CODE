// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase testAuto(DriveSubsystem driveSubsystem)
  {

    double start_time = Timer.getFPGATimestamp();
    Translation2d[] array = {new Translation2d(1,0), new Translation2d(1,2)};
    
    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(driveSubsystem.getPosition(),Arrays.asList(array), new Pose2d(3,3,Rotation2d.fromDegrees(180)), driveSubsystem.getTrajectoryConfig());
    System.out.println(Timer.getFPGATimestamp() - start_time);
    return new RamseteCommand(testTrajectory, driveSubsystem::getPosition, driveSubsystem.getRamseteController(), driveSubsystem.getFeedForward(), driveSubsystem.getDifferentialDriveKinematics(), driveSubsystem::getWheelSpeeds, driveSubsystem.getLeftPIDController(), driveSubsystem.getRightPIDController(), driveSubsystem::setVoltage, driveSubsystem);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
