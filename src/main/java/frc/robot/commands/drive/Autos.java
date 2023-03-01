// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Trajectory;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  private final Subsystem[] subsystems;
  private final DriveSubsystem driveSubsystem;
  private final HashMap<String, Command> eventMap;
  private final RamseteAutoBuilder autoBuilder;

  public Command getMainAutoCommand()
  {
    return getAutoFromPathName("Start Mid Put Cube and Balance");
  }

  public Command getAutoFromPathName(String name) {
    PathPlannerTrajectory path = PathPlanner.loadPath(name, Trajectory.kTrajectoryConfig);
    return getFollowCommand(path);
  }

  public Command getFollowCommand(PathPlannerTrajectory traj) {
    return autoBuilder.fullAuto(traj);
  }

  public Autos(Subsystem[] subsystems, HashMap<String, Command> eMap) {
    eventMap = eMap;
    this.subsystems = subsystems;
    driveSubsystem = (DriveSubsystem) subsystems[0];
    autoBuilder = new RamseteAutoBuilder(driveSubsystem::getPosition, driveSubsystem::resetPosition,
        driveSubsystem.getRamseteController(), driveSubsystem.getDifferentialDriveKinematics(),
        driveSubsystem.getFeedForward(), driveSubsystem::getWheelSpeeds,
        new PIDConstants(Drive.PID.kP, Drive.PID.kI, Drive.PID.kD), driveSubsystem::setVoltage, eventMap, true,
        subsystems);

  }
}
