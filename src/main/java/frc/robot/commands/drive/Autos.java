// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.Constants.Drive;
import frc.robot.Constants.Arm.Physical;
import frc.robot.Constants.CommandGroups.MidCone;
import frc.robot.Constants.CommandGroups.MidCube;

import frc.robot.Constants.Drive.Trajectory;
import frc.robot.commands.Arm.PIDCommands.ArmToConstantAngleCommand;
import frc.robot.commands.Intake.EjectCommand;
import frc.robot.commands.Wrist.WristSetAngleCommand;
import frc.robot.commands.Drive.Auto.AutoBalanceCommand;
import frc.robot.subsystems.Arm.PIDArmSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  private final Subsystem[] subsystems;
  private final DriveSubsystem driveSubsystem;
  private final HashMap<String, Command> eventMap;
  private final RamseteAutoBuilder autoBuilder;
  private final VisionSubsystem visionSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PIDArmSubsystem pidArmSubsystem;
  private final WristSubsystem wristSubsystem;

  public Command getMainAutoCommand() {
    return getAutoFromPathName("Start Mid Put Cone and Balance");
  }

  public Command getAutoFromPathName(String name) {
    PathPlannerTrajectory path = PathPlanner.loadPath(name, Trajectory.kTrajectoryConfig);
    Alliance alliance = DriverStation.getAlliance();
    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);
    return getFollowCommand(path);
  }

  public Command getFollowCommand(PathPlannerTrajectory traj) {
    return autoBuilder.fullAuto(traj);
  }

  public Command getStupidAuto() {
    return (((new ArmToConstantAngleCommand(pidArmSubsystem, MidCube.kArmAngle)
        // .alongWith(new TurnToAngle(m_driveSubsystem, m_visionSubsystem))
        .alongWith(new WristSetAngleCommand(wristSubsystem, MidCube.kWristAngle)))
        .andThen((new EjectCommand(intakeSubsystem, MidCube.kIntakeSpeed)).raceWith(new WaitCommand(2))))
        .raceWith(new WaitCommand(4)))
        .andThen(((new ArmToConstantAngleCommand(pidArmSubsystem, Physical.kArmRestingAngle)
            .alongWith(
                new WristSetAngleCommand(wristSubsystem, frc.robot.Constants.Wrist.Physical.kWristRestingAngle))))
        .raceWith(new WaitCommand(2)));

        // return (new DriveCommand(driveSubsystem, DrivingState.ARCADE, () -> 1, () -> 0, visionSubsystem)
        //     .raceWith(new WaitCommand(1.2)))
        // .andThen(new DriveCommand(driveSubsystem, DrivingState.ARCADE, () -> -1, () -> 0, visionSubsystem)
        //     .raceWith(new WaitCommand(0.5)));
        // .andThen(new AutoBalanceCommand(driveSubsystem));
        // return null;
  }

  public Autos(Subsystem[] subsystems, HashMap<String, Command> eMap) {
    eventMap = eMap;
    this.subsystems = subsystems;
    driveSubsystem = (DriveSubsystem) subsystems[0];
    pidArmSubsystem = (PIDArmSubsystem) subsystems[1];
    wristSubsystem = (WristSubsystem) subsystems[2];
    visionSubsystem = (VisionSubsystem) subsystems[3];
    intakeSubsystem = (IntakeSubsystem) subsystems[4];
    autoBuilder = new RamseteAutoBuilder(driveSubsystem::getPosition, driveSubsystem::resetPosition,
        driveSubsystem.getRamseteController(), driveSubsystem.getDifferentialDriveKinematics(),
        driveSubsystem.getFeedForward(), driveSubsystem::getWheelSpeeds,
        new PIDConstants(Drive.PID.kP, Drive.PID.kI, Drive.PID.kD), driveSubsystem::setVoltage, eventMap, false,
        subsystems);
    Command m_putCubeMid = ((new ArmToConstantAngleCommand(pidArmSubsystem, MidCube.kArmAngle)
        // .alongWith(new TurnToAngle(m_driveSubsystem, m_visionSubsystem))
        .alongWith(new WristSetAngleCommand(wristSubsystem, MidCube.kWristAngle)))
        .andThen(new EjectCommand(intakeSubsystem, MidCube.kIntakeSpeed).deadlineWith(new WaitCommand(2))))
        .andThen((new ArmToConstantAngleCommand(pidArmSubsystem, Physical.kArmRestingAngle)
            .alongWith(
                new WristSetAngleCommand(wristSubsystem, frc.robot.Constants.Wrist.Physical.kWristRestingAngle))));
    Command m_putConeMid = ((new ArmToConstantAngleCommand(pidArmSubsystem, MidCone.kArmAngle)
        // .alongWith(new TurnToAngle(m_driveSubsystem, m_visionSubsystem))
        .alongWith(new WristSetAngleCommand(wristSubsystem, MidCone.kWristAngle)))
        .andThen(new EjectCommand(intakeSubsystem, MidCone.kIntakeSpeed))).deadlineWith(new WaitCommand(2))
        .andThen((new ArmToConstantAngleCommand(pidArmSubsystem, Physical.kArmRestingAngle)
            .alongWith(
                new WristSetAngleCommand(wristSubsystem, frc.robot.Constants.Wrist.Physical.kWristRestingAngle))));
    eMap.replace("Put Cone Mid", m_putConeMid);
    eMap.replace("Put Cube Mid", m_putCubeMid);

  }
}
