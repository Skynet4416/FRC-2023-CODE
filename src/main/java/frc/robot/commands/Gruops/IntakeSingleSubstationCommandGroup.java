package frc.robot.commands.Gruops;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Wrist.WristKeepAngleNoCANCoderCommand;
import frc.robot.commands.Wrist.WristSetAngleCommandNoCANCoder;

public class IntakeSingleSubstationCommandGroup extends SequentialCommandGroup {
    public IntakeSingleSubstationCommandGroup(IntakeCommand intakeCommand, WristSetAngleCommandNoCANCoder wristIntakeAngleCommand, WristKeepAngleNoCANCoderCommand keepAngleNoCANCoderCommand){
        addCommands(wristIntakeAngleCommand,new ParallelCommandGroup(intakeCommand,keepAngleNoCANCoderCommand));
    }
}
