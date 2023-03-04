// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.Autos;
import frc.robot.Constants.Drive;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.Auto.AutoBalanceCommand;
import frc.robot.commands.drive.Auto.TurnToAngle;
import frc.robot.commands.drive.Auto.TurnToConstantAngle;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.subsystems.Arm.PIDArmSubsystem;
import frc.robot.subsystems.Arm.StateSpacedArmSubsystem;
import frc.robot.Constants.Arm.Physical;
import frc.robot.Constants.Arm.PoistionPID.PID;
import frc.robot.Constants.CommandGroups.HighCone;
import frc.robot.Constants.CommandGroups.HighCube;
import frc.robot.Constants.CommandGroups.IntakeGround;
import frc.robot.Constants.CommandGroups.IntakeSubstation;
import frc.robot.Constants.CommandGroups.LowCube;
import frc.robot.Constants.CommandGroups.MidCone;
import frc.robot.Constants.CommandGroups.MidCube;
import frc.robot.Constants.Drive.ChargeStationPID;
import frc.robot.Constants.Drive.PIDAngular;
import frc.robot.commands.Arm.MovePrecentageCommand;
import frc.robot.commands.Arm.PIDCommands.ArmInstantCommand;
import frc.robot.commands.Arm.PIDCommands.ArmKeepAtConstantAngle;
import frc.robot.commands.Arm.PIDCommands.ArmToConstantAngleCommand;
import frc.robot.commands.Arm.StateSpaceCommands.ArmToConstantHeightCommand;
import frc.robot.commands.Intake.ConstantIntake;
import frc.robot.commands.Intake.EjectCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Wrist.WristKeepAngleCommand;
import frc.robot.commands.Wrist.WristSetAngleCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(false);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_visionSubsystem);
  private final WristSubsystem m_WristSubsystem = new WristSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final PIDArmSubsystem m_PidArmSubsystem = new PIDArmSubsystem();
  private final OI oi = new OI();
  private final Autos m_autos;
  private final Command m_wrestingCommand = (new ArmToConstantAngleCommand(m_PidArmSubsystem, Physical.kArmRestingAngle)
      .alongWith(new WaitCommand(1).andThen(new WristSetAngleCommand(m_WristSubsystem, frc.robot.Constants.Wrist.Physical.kWristRestingAngle))));
  private final HashMap<String, Command> commandHashMap = new HashMap<String, Command>();
  private final Command m_PutConeOrCubeLow = (new ArmToConstantAngleCommand(m_PidArmSubsystem, LowCube.kArmAngle)
      // .alongWith(new TurnToAngle(m_driveSubsystem, m_visionSubsystem))
      .alongWith(new WristSetAngleCommand(m_WristSubsystem, LowCube.kWristAngle)))
      .andThen(new EjectCommand(m_intakeSubsystem, MidCone.kIntakeSpeed)
          .alongWith(new WristKeepAngleCommand(m_WristSubsystem))
          .alongWith(new ArmKeepAtConstantAngle(m_PidArmSubsystem)));
  private final Command m_putCubeMid = (new ArmToConstantAngleCommand(m_PidArmSubsystem, MidCube.kArmAngle)
      // .alongWith(new TurnToAngle(m_driveSubsystem, m_visionSubsystem))
      .alongWith(new WristSetAngleCommand(m_WristSubsystem, MidCube.kWristAngle)))
      .andThen(new EjectCommand(m_intakeSubsystem, MidCone.kIntakeSpeed)
          .alongWith(new WristKeepAngleCommand(m_WristSubsystem))
          .alongWith(new ArmKeepAtConstantAngle(m_PidArmSubsystem)));
  private final Command m_putConeMid = (new ArmToConstantAngleCommand(m_PidArmSubsystem, MidCone.kArmAngle)
      // .alongWith(new TurnToAngle(m_driveSubsystem, m_visionSubsystem))
      .alongWith(new WristSetAngleCommand(m_WristSubsystem, MidCone.kWristAngle)))
      .andThen((new ArmToConstantAngleCommand(m_PidArmSubsystem, MidCone.kArmAngle2)
          .alongWith(new WristSetAngleCommand(m_WristSubsystem, MidCone.kWristAngle2))))
      .andThen(new EjectCommand(m_intakeSubsystem, MidCone.kIntakeSpeed)
          .alongWith(new WristKeepAngleCommand(m_WristSubsystem))
          .alongWith(new ArmKeepAtConstantAngle(m_PidArmSubsystem)));
  private final Command m_putCubeHigh = (new ArmToConstantAngleCommand(m_PidArmSubsystem, HighCube.kArmAngle)
      // .alongWith(new TurnToAngle(m_driveSubsystem, m_visionSubsystem))
      .alongWith(new WristSetAngleCommand(m_WristSubsystem, HighCube.kWristAngle)))
      .andThen(new EjectCommand(m_intakeSubsystem, MidCone.kIntakeSpeed)
          .alongWith(new WristKeepAngleCommand(m_WristSubsystem))
          .alongWith(new ArmKeepAtConstantAngle(m_PidArmSubsystem)));
  private final Command m_autoBalacne = new AutoBalanceCommand(m_driveSubsystem);

  // // Replace with CommandPS4Controller or CommandJoystick if needed

  // private final StateSpacedArmSubsystem m_stateSpaceArmSubsystem = new
  // StateSpacedArmSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws IOException {
    // Configure the trigger bindings
    configureBindings();
    configureSmartDashboard();
    LiveWindow.disableAllTelemetry();
    CreateHashMap();
    m_autos = new Autos(
        new Subsystem[] { m_driveSubsystem, m_PidArmSubsystem, m_WristSubsystem, m_visionSubsystem, m_intakeSubsystem },
        commandHashMap);
  }

  public void CreateHashMap() {
    commandHashMap.put("Put Cone Low", m_PutConeOrCubeLow);
    commandHashMap.put("Put Cube Low", m_PutConeOrCubeLow);
    commandHashMap.put("Put Cube Mid", m_putCubeMid);
    commandHashMap.put("Put Cone Mid", m_putConeMid);
    commandHashMap.put("Put Cube Hight", m_putCubeHigh);
    commandHashMap.put("Auto Balance", m_autoBalacne);
    commandHashMap.put("Return Arm", m_wrestingCommand);

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
  private void configureSmartDashboard() {
    SmartDashboard.putNumber("Drive Angular kp", PIDAngular.kP);
    SmartDashboard.putNumber("Arm P", PID.kP);
    SmartDashboard.putNumber("Wanted Arm setpoint", 0);
    SmartDashboard.putNumber("Arm I", PID.kI);
    SmartDashboard.putNumber("Arm D", PID.kD);
    SmartDashboard.putNumber("Drive Angular kp", 0);
    SmartDashboard.putNumber("ChargeStation d", ChargeStationPID.kD);
    SmartDashboard.putNumber("ChargeStation p", ChargeStationPID.kP);
    SmartDashboard.putNumber("ChargeStation i", ChargeStationPID.kI);

  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem, Drive.kDriveState,
        oi.rightJoystickController::getY, oi.leftJoystickController::getY, m_visionSubsystem));
    // oi.rightJoystickController::getX, oi.rightJoystickController::getX));
    // OI.A.onTrue(new TurnToConstantAngle(m_driveSubsystem, 30));
    m_WristSubsystem.setDefaultCommand(new WristKeepAngleCommand(m_WristSubsystem));
    m_PidArmSubsystem.setDefaultCommand(new ArmKeepAtConstantAngle(m_PidArmSubsystem));
    m_intakeSubsystem.setDefaultCommand(new ConstantIntake(m_intakeSubsystem));
    // oi.B.onTrue(new ArmToConstantAngleCommand(m_PidArmSubsystem, -90));
    // oi.X.onTrue(new ArmToConstantAngleCommand(m_PidArmSubsystem, 30));

    // oi.A.onTrue(new WristSetAngleCommand(m_WristSubsystem, 90));
    // OI.X.onTrue(new ArmToConstantAngleCommand(m_PidArmSubsystem, 30.0));
    // OI.X.onTrue(new InstantCommand(
    // () -> m_PidArmSubsystem.setAngleInDegrees(SmartDashboard.getNumber("Wanted
    // Arm setpoint", 0))));
    oi.DPadDOWN.whileTrue(m_PutConeOrCubeLow);
    oi.DPadDOWN.onFalse(m_wrestingCommand);
    oi.DPadUP.onFalse(m_wrestingCommand);
    oi.DPadLEFT.onFalse(m_wrestingCommand);
    oi.DpadRIGHT.onFalse(m_wrestingCommand);
    oi.A.onFalse(m_wrestingCommand);
    oi.Y.onFalse(m_wrestingCommand);
    oi.DPadLEFT.whileTrue(m_putCubeMid);
    oi.DpadRIGHT.whileTrue(m_putConeMid);
    oi.DPadUP.whileTrue(m_putCubeHigh);
    oi.A.whileTrue((new ArmToConstantAngleCommand(m_PidArmSubsystem,
        IntakeGround.kArmAngle)
        .alongWith(new WristSetAngleCommand(m_WristSubsystem,
            IntakeGround.kWristAngle)))
        .andThen(new IntakeCommand(m_intakeSubsystem, IntakeGround.kIntakeSpeed)
            .alongWith(new WristKeepAngleCommand(m_WristSubsystem))
            .alongWith(new ArmKeepAtConstantAngle(m_PidArmSubsystem))));
    oi.Y.whileTrue((new ArmToConstantAngleCommand(m_PidArmSubsystem,
        IntakeSubstation.kArmAngle)
        .alongWith(new WristSetAngleCommand(m_WristSubsystem,
            IntakeSubstation.kWristAngle)))
        .andThen(new IntakeCommand(m_intakeSubsystem, IntakeSubstation.kIntakeSpeed)
            .alongWith(new WristKeepAngleCommand(m_WristSubsystem))
            .alongWith(new ArmKeepAtConstantAngle(m_PidArmSubsystem))));
    oi.B.onTrue(m_autoBalacne);

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
    // return
    return m_autos.getMainAutoCommand();
    // return m_autos.getStupidAuto();
    // return null;

  }

  public void enableBreak() {
    m_driveSubsystem.setIdleMode(IdleMode.kBrake);
  }

  public void disableBreak() {
    m_driveSubsystem.setIdleMode(IdleMode.kCoast);
  }
}