package frc.robot.commands.Gruops;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.EjectCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Wrist.WristKeepAngleNoCANCoderCommand;
import frc.robot.commands.Wrist.WristSetAngleCommandNoCANCoder;

public class PutConeOrCubeLowCommandGroup extends SequentialCommandGroup{
    public PutConeOrCubeLowCommandGroup(IntakeCommand constantIntake, WristSetAngleCommandNoCANCoder wristMidPoint,
             WristSetAngleCommandNoCANCoder wristConeFinalPosCommand,
            WristKeepAngleNoCANCoderCommand wristKeepAngleNoCANCoderCommand, EjectCommand ejectCommand) {
        addCommands(new ParallelRaceGroup(
                new SequentialCommandGroup(wristMidPoint, wristConeFinalPosCommand),
                constantIntake), new ParallelCommandGroup(wristKeepAngleNoCANCoderCommand, ejectCommand));
    }
}
