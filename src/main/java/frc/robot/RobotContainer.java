// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.Autos;
import frc.robot.Constants.Drive;
import frc.robot.commands.drive.DriveCommand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.io.IOException;

import frc.robot.subsystems.Arm.StateSpacedArmSubsystem;
import frc.robot.Constants.Arm.Physical;
import frc.robot.commands.Arm.StateSpaceCommands.ArmToConstantHeightCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final StateSpacedArmSubsystem m_stateSpaceArmSubsystem = new StateSpacedArmSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws IOException {
    // Configure the trigger bindings
    configureBindings();
    LiveWindow.disableAllTelemetry();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem, Drive.kDriveState,
        OI.xboxController::getLeftY, OI.xboxController::getRightY));

    OI.A.whileTrue(new IntakeCommand(m_intakeSubsystem, 1));
    m_stateSpaceArmSubsystem.setDefaultCommand(
        new frc.robot.commands.Arm.StateSpaceCommands.KeepArmAtStateCommand(m_stateSpaceArmSubsystem));
    OI.DPadDOWN
        .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kGroundGridHeight));
    OI.DPadLEFT
        .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kCubeMidGridHeight));
    OI.DpadRIGHT
        .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kConeMidGridHeight));
    OI.DPadUP
        .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kCubeHighGridHeight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {
    // An example command will be run in autonomous

    // Run path following command, then stop at the end.
    return Autos.testAuto(m_driveSubsystem).andThen(() -> m_driveSubsystem.setVoltage(0, 0));
  }
}