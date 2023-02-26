// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.Autos;
import frc.robot.Constants.Drive;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.Auto.TurnToAngle;
import frc.robot.commands.drive.Auto.TurnToConstantAngle;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.subsystems.Arm.PIDArmSubsystem;
import frc.robot.subsystems.Arm.StateSpacedArmSubsystem;
import frc.robot.Constants.Arm.Physical;
import frc.robot.Constants.Arm.PoistionPID.PID;
import frc.robot.Constants.Drive.PIDAngular;
import frc.robot.commands.Arm.MovePrecentageCommand;
import frc.robot.commands.Arm.PIDCommands.ArmKeepAtConstantAngle;
import frc.robot.commands.Arm.PIDCommands.ArmToConstantAngleCommand;
import frc.robot.commands.Arm.StateSpaceCommands.ArmToConstantHeightCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Wrist.WristKeepAngleCommand;
import frc.robot.commands.Wrist.WristSetAngleCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final WristSubsystem m_WristSubsystem = new WristSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final PIDArmSubsystem m_PidArmSubsystem = new PIDArmSubsystem();
  // // Replace with CommandPS4Controller or CommandJoystick if needed

  // private final StateSpacedArmSubsystem m_stateSpaceArmSubsystem = new StateSpacedArmSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws IOException {
    // Configure the trigger bindings
    configureBindings();
    configureSmartDashboard();
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
  private void configureSmartDashboard()
  {
    SmartDashboard.putNumber("Drive Angular kp", PIDAngular.kP);
    SmartDashboard.putNumber("Arm P", PID.kP);
    SmartDashboard.putNumber("Wanted Arm setpoint", 0);
    SmartDashboard.putNumber("Arm I", PID.kI);
    SmartDashboard.putNumber("Arm D", PID.kD);
  }
  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem, Drive.kDriveState,
        OI.xboxController::getLeftY, OI.xboxController::getRightY));
    // OI.A.onTrue(new TurnToConstantAngle(m_driveSubsystem, 30));
    m_WristSubsystem.setDefaultCommand(new WristKeepAngleCommand(m_WristSubsystem));
    m_PidArmSubsystem.setDefaultCommand(new ArmKeepAtConstantAngle(m_PidArmSubsystem));
    // OI.A.onTrue(new WristSetAngleCommand(m_WristSubsystem, 0));
    // OI.X.onTrue(new ArmToConstantAngleCommand(m_PidArmSubsystem, 30.0));
    OI.X.onTrue(new InstantCommand(() -> m_PidArmSubsystem.setAngleInDegrees(SmartDashboard.getNumber("Wanted Arm setpoint", 0))));
    OI.B.onTrue(new InstantCommand(() ->m_PidArmSubsystem.lockArm()));
    OI.Y.onTrue(new InstantCommand(() ->m_PidArmSubsystem.unlockArm()));



    
    // OI.B.whileTrue(new IntakeCommand(m_intakeSubsystem, 0.5));
    // OI.A.whileTrue(new MovePrecentageCommand(m_PidArmSubsystem, -0.1));
    // OI.X.whileTrue(new MovePrecentageCommand(m_PidArmSubsystem, 0.1));

    // OI.A.whileTrue(new IntakeCommand(m_intakeSubsystem, 1));
    // m_stateSpaceArmSubsystem.setDefaultCommand(
    //     new frc.robot.commands.Arm.StateSpaceCommands.KeepArmAtStateCommand(m_stateSpaceArmSubsystem));
    // OI.DPadDOWN
    //     .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kGroundGridHeight));
    // OI.DPadLEFT
    //     .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kCubeMidGridHeight));
    // OI.DpadRIGHT
    //     .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kConeMidGridHeight));
    // OI.DPadUP
    //     .onTrue(new ArmToConstantHeightCommand(m_stateSpaceArmSubsystem, Physical.kCubeHighGridHeight));
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
  public void enableBreak()
  {
    m_driveSubsystem.setIdleMode(IdleMode.kBrake);
  }
  public void disableBreak()
  {
    m_driveSubsystem.setIdleMode(IdleMode.kCoast);
  }
}