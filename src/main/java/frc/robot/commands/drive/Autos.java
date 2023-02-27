// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.Drive.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Paths;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static CommandBase testAuto(DriveSubsystem driveSubsystem) throws IOException {

    Trajectory testTrajectory =  TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/Start Mid Put Cone and Balance.json"));
    return new RamseteCommand(testTrajectory, driveSubsystem::getPosition, driveSubsystem.getRamseteController(),
        driveSubsystem.getFeedForward(), driveSubsystem.getDifferentialDriveKinematics(),
        driveSubsystem::getWheelSpeeds, driveSubsystem.getLeftPIDController(), driveSubsystem.getRightPIDController(),
        driveSubsystem::setVoltage, driveSubsystem);
  }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
