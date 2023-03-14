// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Drive;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.Auto.AutoBalanceCommand;
import frc.robot.commands.drive.Auto.AutoManager;
import frc.robot.commands.drive.Auto.GamePice;
import frc.robot.commands.drive.Auto.StartingPosition;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.Arm.PoistionPID.PID;
import frc.robot.Constants.CommandGroups.HighCone;
import frc.robot.Constants.CommandGroups.HighCube;

import frc.robot.Constants.CommandGroups.LowCube;
import frc.robot.Constants.CommandGroups.MidCone;
import frc.robot.Constants.CommandGroups.MidCube;
import frc.robot.Constants.CommandGroups.MidPoint;
import frc.robot.Constants.CommandGroups.SingleSubstation;
import frc.robot.Constants.Drive.ChargeStationPID;
import frc.robot.Constants.Drive.PIDAngular;
import frc.robot.commands.Elevator.ElevatorCloseCommand;
import frc.robot.commands.Elevator.ElevatorOpenCommand;
import frc.robot.commands.Gruops.IntakeSingleSubstationCommandGroup;
import frc.robot.commands.Gruops.PutConeMidCommandGroup;
import frc.robot.commands.Gruops.PutConeOrCubeLowCommandGroup;
import frc.robot.commands.Gruops.PutCubeHighCommandGroup;
import frc.robot.commands.Gruops.PutCubeMidCommandGroup;
import frc.robot.commands.Gruops.RestingCommandGroup;
import frc.robot.commands.Intake.EjectCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Wrist.WristKeepAngleNoCANCoderCommand;
import frc.robot.commands.Wrist.WristSetAngleCommandNoCANCoder;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LED.ColorEnum;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Wrist.WristNoCANCoder;
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
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(false);
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_visionSubsystem);
    private final WristNoCANCoder m_WristSubsystem = new WristNoCANCoder();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
    // private final PIDArmSubsystem m_PidArmSubsystem = new PIDArmSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final OI oi = new OI();
    private final ElevatorCloseCommand elevatorCloseCommand = new ElevatorCloseCommand(m_elevatorSubsystem);
    private final ElevatorOpenCommand elevatorOpenCommand = new ElevatorOpenCommand(m_elevatorSubsystem);
    private final WristSetAngleCommandNoCANCoder midPointWristCommand = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, MidPoint.kWristAngle);
    private final WristSetAngleCommandNoCANCoder restingPointWristCommand = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, 0);
    private final IntakeCommand consIntakeCommand = new IntakeCommand(m_intakeSubsystem, 0.1);
    private final WristSetAngleCommandNoCANCoder coneOrCubeLowFinalPostion = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, LowCube.kWristAngle);
    private final WristKeepAngleNoCANCoderCommand wristKeepAngleNoCANCoderCommand = new WristKeepAngleNoCANCoderCommand(
            m_WristSubsystem);
    private final EjectCommand lowEjectCommand = new EjectCommand(m_intakeSubsystem, LowCube.kIntakeSpeed);
    private final WristSetAngleCommandNoCANCoder coneMidWristPosition = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, MidCone.kWristAngle);
    private final EjectCommand midConeEjectCommand = new EjectCommand(m_intakeSubsystem, MidCone.kIntakeSpeed);
    private final WristSetAngleCommandNoCANCoder highCubeWristPositionCommand = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, HighCube.kWristAngle);
    private final WristSetAngleCommandNoCANCoder highConeWristCommand = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, HighCone.kWristAngle);
    private final EjectCommand highCubeEjectCommand = new EjectCommand(m_intakeSubsystem, HighCube.kIntakeSpeed);
    private final WristSetAngleCommandNoCANCoder cubeMidPosition = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, MidCube.kWristAngle);
    private final EjectCommand highConeEjectCommand = new EjectCommand(m_intakeSubsystem, HighCone.kIntakeSpeed);

    private final EjectCommand midCubeEjectCommand = new EjectCommand(m_intakeSubsystem, MidCube.kIntakeSpeed);
    private final IntakeCommand singleSubstationIntakecommand = new IntakeCommand(m_intakeSubsystem,
            SingleSubstation.kIntakeSpeed);
    private final WristSetAngleCommandNoCANCoder singleSubstationWristPositioncommand = new WristSetAngleCommandNoCANCoder(
            m_WristSubsystem, SingleSubstation.kWristAngle);
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    // command Groups
    private final IntakeSingleSubstationCommandGroup m_IntakeSingleSubstationCommandGroup = new IntakeSingleSubstationCommandGroup(
            singleSubstationIntakecommand, singleSubstationWristPositioncommand, wristKeepAngleNoCANCoderCommand);
    private final RestingCommandGroup m_restingCommand = new RestingCommandGroup(midPointWristCommand,
            restingPointWristCommand, elevatorCloseCommand, new IntakeCommand(consIntakeCommand));

    private final Command m_PutConeOrCubeLow = new PutConeOrCubeLowCommandGroup(consIntakeCommand,
            new WristSetAngleCommandNoCANCoder(midPointWristCommand),
            coneOrCubeLowFinalPostion, new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand),
            lowEjectCommand);

    private final Command m_putConeMid = new PutConeMidCommandGroup(new IntakeCommand(consIntakeCommand),
            new WristSetAngleCommandNoCANCoder(midPointWristCommand),
            elevatorOpenCommand, coneMidWristPosition,
            new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand),
            midConeEjectCommand);
    private final Command m_putCubeHigh = new PutCubeHighCommandGroup(new IntakeCommand(consIntakeCommand),
            new WristSetAngleCommandNoCANCoder(midPointWristCommand),
            new ElevatorOpenCommand(elevatorOpenCommand), highCubeWristPositionCommand,
            new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand), highCubeEjectCommand);
    private final Command m_putCubeMidCommand = new PutCubeMidCommandGroup(new IntakeCommand(consIntakeCommand),
            new WristSetAngleCommandNoCANCoder(midPointWristCommand),
            new ElevatorOpenCommand(elevatorOpenCommand), cubeMidPosition,
            new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand), midCubeEjectCommand);
    private final Command m_putConeHigh = new PutCubeHighCommandGroup(new IntakeCommand(consIntakeCommand),
            new WristSetAngleCommandNoCANCoder(midPointWristCommand),
            new ElevatorOpenCommand(elevatorOpenCommand), highConeWristCommand,
            new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand), highConeEjectCommand);
    private final Command m_autoBalacne = new AutoBalanceCommand(m_driveSubsystem);
    private final AutoManager m_autoManager = new AutoManager(m_driveSubsystem, m_WristSubsystem, m_intakeSubsystem,
            m_elevatorSubsystem, m_visionSubsystem);
    private final HashMap<String, Command> commandHashMap = new HashMap<String, Command>();

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
        // m_autos = new Autos(
        // new Subsystem[] { m_driveSubsystem, m_WristSubsystem, m_visionSubsystem,
        // m_intakeSubsystem },
        // commandHashMap);
    }

    public void CreateHashMap() {
        commandHashMap.put("Put Cone Low", m_PutConeOrCubeLow);
        commandHashMap.put("Put Cube Low", m_PutConeOrCubeLow);
        commandHashMap.put("Put Cube Mid", m_putCubeMidCommand);
        commandHashMap.put("Put Cone Mid", m_putConeMid);
        commandHashMap.put("Put Cube Hight", m_putCubeHigh);
        commandHashMap.put("Auto Balance", m_autoBalacne);
        commandHashMap.put("Return Arm", m_restingCommand);

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
        SmartDashboard.putNumber("wanted wrist", m_WristSubsystem.getWristAngleInDegrees());
        m_chooser.setDefaultOption("Cube Charging Station",
                m_autoManager.getCommand(GamePice.CUBE, StartingPosition.IN_FRONT_OF_CHARGING_STATION));
        m_chooser.addOption("Cone Charging Station",
                m_autoManager.getCommand(GamePice.CONE, StartingPosition.IN_FRONT_OF_CHARGING_STATION));
        m_chooser.addOption("Cone Cable Gaurd",
                m_autoManager.getCommand(GamePice.CONE, StartingPosition.IN_FRONT_OF_CABLE_GAURD));
        m_chooser.addOption("Cube Cable Gaurd",
                m_autoManager.getCommand(GamePice.CUBE, StartingPosition.IN_FRONT_OF_CABLE_GAURD));
        m_chooser.addOption("Cone Substation",
                m_autoManager.getCommand(GamePice.CONE, StartingPosition.IN_FRONT_OF_SINGLE_SUBSTATION));
        m_chooser.addOption("Cube Substation",
                m_autoManager.getCommand(GamePice.CUBE, StartingPosition.IN_FRONT_OF_SINGLE_SUBSTATION));
        m_chooser.addOption("Cube",
                m_autoManager.getCommand(GamePice.CUBE, StartingPosition.NONE));
        SmartDashboard.putData(m_chooser);
    }

    private void configureBindings() {
        m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem, Drive.kDriveState,
                oi.leftJoystickController::getY, oi.rightJoystickController::getY, m_visionSubsystem));
        m_WristSubsystem.setDefaultCommand(new WristKeepAngleNoCANCoderCommand(m_WristSubsystem));

        oi.DPadDOWN.whileTrue(m_PutConeOrCubeLow);
        oi.DPadDOWN.onFalse(m_restingCommand);
        oi.DPadUP.onFalse(m_restingCommand);
        oi.DPadLEFT.onFalse(m_restingCommand);
        oi.DpadRIGHT.onFalse(m_restingCommand);
        oi.A.onFalse(m_restingCommand);
        oi.DPadLEFT.whileTrue(m_putCubeMidCommand);
        oi.DpadRIGHT.whileTrue(m_putConeMid);
        oi.DPadUP.whileTrue(m_putCubeHigh);
        oi.RightBumper.whileTrue(new InstantCommand(() -> m_LEDSubsystem.setColor(ColorEnum.YELLOW), m_LEDSubsystem));
        oi.LeftBumper.whileTrue(new InstantCommand(() -> m_LEDSubsystem.setColor(ColorEnum.PERPULE), m_LEDSubsystem));
        oi.A.whileTrue(m_IntakeSingleSubstationCommandGroup);
        oi.Y.whileTrue(m_putConeHigh);
        // oi.A.whileTrue(new ElevatorOpenCommand(m_elevatorSubsystem));
        // oi.B.whileTrue(new ElevatorCloseCommand(m_elevatorSubsystem));

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
        return m_chooser.getSelected();

    }

    public void enableBreak() {
        m_driveSubsystem.setIdleMode(IdleMode.kBrake);
    }

    public void disableBreak() {
        m_driveSubsystem.setIdleMode(IdleMode.kCoast);
    }
}