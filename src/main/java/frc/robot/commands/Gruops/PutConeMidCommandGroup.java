package frc.robot.commands.Gruops;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.ElevatorOpenCommand;
import frc.robot.commands.Intake.EjectCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Wrist.WristKeepAngleNoCANCoderCommand;
import frc.robot.commands.Wrist.WristSetAngleCommandNoCANCoder;

public class PutConeMidCommandGroup extends SequentialCommandGroup {
    public PutConeMidCommandGroup(IntakeCommand constantIntake, WristSetAngleCommandNoCANCoder wristMidPoint,
            ElevatorOpenCommand elevatorOpenCommand, WristSetAngleCommandNoCANCoder wristConeFinalPosCommand,
            WristKeepAngleNoCANCoderCommand wristKeepAngleNoCANCoderCommand, EjectCommand ejectCommand) {
        addCommands(new ParallelRaceGroup(
                new SequentialCommandGroup(wristMidPoint, elevatorOpenCommand, wristConeFinalPosCommand),
                constantIntake), new ParallelCommandGroup(wristKeepAngleNoCANCoderCommand, ejectCommand));
    }

}
