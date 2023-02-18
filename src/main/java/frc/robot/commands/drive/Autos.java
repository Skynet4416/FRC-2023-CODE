// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.Drive.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static CommandBase testAuto(DriveSubsystem driveSubsystem) throws IOException {

    Trajectory testTrajectory = TrajectoryUtil
        .fromPathweaverJson(Paths.get("src\\main\\deploy\\pathplanner\\generatedJSON\\New Path.wpilib.json"));
    return new RamseteCommand(testTrajectory, driveSubsystem::getPosition, driveSubsystem.getRamseteController(),
        driveSubsystem.getFeedForward(), driveSubsystem.getDifferentialDriveKinematics(),
        driveSubsystem::getWheelSpeeds, driveSubsystem.getLeftPIDController(), driveSubsystem.getRightPIDController(),
        driveSubsystem::setVoltage, driveSubsystem);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}