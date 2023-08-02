package frc.robot.commands.Drive.Auto;

import frc.robot.Constants.Auto;
import frc.robot.commands.Drive.DrivingState;
import frc.robot.Constants.CommandGroups.HighCube;

import frc.robot.Constants.CommandGroups.LowCube;
import frc.robot.Constants.CommandGroups.MidCone;
import frc.robot.Constants.CommandGroups.MidCube;
import frc.robot.Constants.CommandGroups.MidPoint;
import frc.robot.Constants.CommandGroups.SingleSubstation;
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
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Wrist.WristNoCANCoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoManager {
        private final DriveSubsystem driveSubsystem;
        private final WristNoCANCoder wristSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ElevatorSubsystem elevatorSubsystem;
        private final VisionSubsystem visionSubsystem;

        public AutoManager(DriveSubsystem driveSubsystem, WristNoCANCoder wristSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
                this.driveSubsystem = driveSubsystem;
                this.wristSubsystem = wristSubsystem;
                this.elevatorSubsystem = elevatorSubsystem;
                this.intakeSubsystem = intakeSubsystem;
                this.visionSubsystem = visionSubsystem;
        }

        public Command getChargingStation(GamePice gamePice) {

                return getGamePice(gamePice)
                                .andThen(new ParallelDeadlineGroup(new WaitCommand(Auto.ChargingStation.Forward.kTime),
                                                new AutoDriveCommand(driveSubsystem, DrivingState.ARCADE,
                                                                Auto.ChargingStation.Forward.kSpeed, 0,
                                                                visionSubsystem)))
                                .andThen(new ParallelDeadlineGroup(
                                                new WaitCommand(Auto.ChargingStation.Forward.kTime2),
                                                new AutoDriveCommand(driveSubsystem, DrivingState.ARCADE,
                                                                Auto.ChargingStation.Forward.kSpeed2, 0,
                                                                visionSubsystem)))
                                .andThen(new ParallelDeadlineGroup(
                                                new WaitCommand(Auto.ChargingStation.Backwords.kTime),
                                                new AutoDriveCommand(driveSubsystem, DrivingState.ARCADE,
                                                                Auto.ChargingStation.Backwords.kSpeed, 0,
                                                                visionSubsystem)));
                                // .andThen(new AutoBalanceCommand(driveSubsystem, false));

        }

        public Command getCommand(GamePice gamePice, StartingPosition startingPosition) {

                switch (startingPosition) {
                        case IN_FRONT_OF_CHARGING_STATION:
                                return getChargingStation(gamePice);
                        case IN_FRONT_OF_CABLE_GAURD:
                                return getFrontOfCableGaurd(gamePice);
                        case IN_FRONT_OF_SINGLE_SUBSTATION:
                                return getInFrontOfSubstation(gamePice);
                        default:
                                return getGamePice(gamePice);

                }
        }

        private Command getGamePice(GamePice gamePice) {
                ElevatorCloseCommand elevatorCloseCommand = new ElevatorCloseCommand(elevatorSubsystem);
                ElevatorOpenCommand elevatorOpenCommand = new ElevatorOpenCommand(elevatorSubsystem);
                WristSetAngleCommandNoCANCoder midPointWristCommand = new WristSetAngleCommandNoCANCoder(
                                wristSubsystem, MidPoint.kWristAngle);
                WristSetAngleCommandNoCANCoder restingPointWristCommand = new WristSetAngleCommandNoCANCoder(
                                wristSubsystem, 0);
                IntakeCommand consIntakeCommand = new IntakeCommand(intakeSubsystem, 0.18);
                WristSetAngleCommandNoCANCoder coneOrCubeLowFinalPostion = new WristSetAngleCommandNoCANCoder(
                                wristSubsystem, LowCube.kWristAngle);
                WristKeepAngleNoCANCoderCommand wristKeepAngleNoCANCoderCommand = new WristKeepAngleNoCANCoderCommand(
                                wristSubsystem);
                EjectCommand lowEjectCommand = new EjectCommand(intakeSubsystem, LowCube.kIntakeSpeed);
                WristSetAngleCommandNoCANCoder coneMidWristPosition = new WristSetAngleCommandNoCANCoder(
                                wristSubsystem, MidCone.kWristAngle);
                EjectCommand midConeEjectCommand = new EjectCommand(intakeSubsystem, MidCone.kIntakeSpeed);
                WristSetAngleCommandNoCANCoder highCubeWristPositionCommand = new WristSetAngleCommandNoCANCoder(
                                wristSubsystem, HighCube.kWristAngle);
                EjectCommand highCubeEjectCommand = new EjectCommand(intakeSubsystem, HighCube.kIntakeSpeed);
                WristSetAngleCommandNoCANCoder cubeMidPosition = new WristSetAngleCommandNoCANCoder(
                                wristSubsystem, MidCube.kWristAngle);
                EjectCommand midCubeEjectCommand = new EjectCommand(intakeSubsystem, MidCube.kIntakeSpeed);
                IntakeCommand singleSubstationIntakecommand = new IntakeCommand(intakeSubsystem,
                                SingleSubstation.kIntakeSpeed);
                WristSetAngleCommandNoCANCoder singleSubstationWristPositioncommand = new WristSetAngleCommandNoCANCoder(
                                wristSubsystem, SingleSubstation.kWristAngle);

                // command Groups
                IntakeSingleSubstationCommandGroup IntakeSingleSubstationCommandGroup = new IntakeSingleSubstationCommandGroup(
                                singleSubstationIntakecommand, singleSubstationWristPositioncommand,
                                wristKeepAngleNoCANCoderCommand);
                RestingCommandGroup restingCommand = new RestingCommandGroup(midPointWristCommand,
                                restingPointWristCommand, elevatorCloseCommand, new IntakeCommand(consIntakeCommand));

                Command PutConeOrCubeLow = new PutConeOrCubeLowCommandGroup(consIntakeCommand,
                                new WristSetAngleCommandNoCANCoder(midPointWristCommand),
                                coneOrCubeLowFinalPostion,
                                new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand),
                                lowEjectCommand);

                Command putConeMid = new PutConeMidCommandGroup(new IntakeCommand(consIntakeCommand),
                                new WristSetAngleCommandNoCANCoder(midPointWristCommand),
                                elevatorOpenCommand, coneMidWristPosition,
                                new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand),
                                midConeEjectCommand);
                Command putCubeHigh = new PutCubeHighCommandGroup(new IntakeCommand(consIntakeCommand),
                                new WristSetAngleCommandNoCANCoder(midPointWristCommand),
                                new ElevatorOpenCommand(elevatorOpenCommand), highCubeWristPositionCommand,
                                new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand),
                                highCubeEjectCommand);
                Command putCubeMidCommand = new PutCubeMidCommandGroup(new IntakeCommand(consIntakeCommand),
                                new WristSetAngleCommandNoCANCoder(midPointWristCommand),
                                new ElevatorOpenCommand(elevatorOpenCommand), cubeMidPosition,
                                new WristKeepAngleNoCANCoderCommand(wristKeepAngleNoCANCoderCommand),
                                midCubeEjectCommand);
                Command autoBalacne = new AutoBalanceCommand(driveSubsystem, true);
                switch (gamePice) {
                        case CUBE:
                                return new ParallelRaceGroup(
                                                new AutoDriveCommand(driveSubsystem, DrivingState.TANK, 0, 0,
                                                                visionSubsystem),
                                                new SequentialCommandGroup(
                                                                new ParallelDeadlineGroup(new WaitCommand(5),
                                                                putCubeHigh),
                                                                restingCommand));
                        case CONE:
                        return new ParallelRaceGroup(
                                new AutoDriveCommand(driveSubsystem, DrivingState.TANK, 0, 0,
                                                visionSubsystem),
                                new SequentialCommandGroup(
                                                new ParallelDeadlineGroup(new WaitCommand(5),
                                                putConeMid),
                                                restingCommand));
                        default:
                                return new InstantCommand();

                }
        }

        private Command getFrontOfCableGaurd(GamePice gamePice) {
                return getGamePice(gamePice)
                                .andThen(new ParallelDeadlineGroup(new WaitCommand(Auto.CableGaurd.Forward.kTime),
                                                new AutoDriveCommand(driveSubsystem, DrivingState.ARCADE,
                                                                Auto.CableGaurd.Forward.kSpeed, 0, visionSubsystem)));
        }

        private Command getInFrontOfSubstation(GamePice gamePice) {
                return getGamePice(gamePice)
                                .andThen(new ParallelDeadlineGroup(new WaitCommand(Auto.Substation.Forward.kTime),
                                                new AutoDriveCommand(driveSubsystem, DrivingState.ARCADE,
                                                                Auto.Substation.Forward.kSpeed, 0, visionSubsystem)));
        }
}