package frc.robot.commands.Gruops;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.ElevatorCloseCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Wrist.WristSetAngleCommandNoCANCoder;

public class RestingCommandGroup extends ParallelRaceGroup{
    public RestingCommandGroup(WristSetAngleCommandNoCANCoder midPointCommand, WristSetAngleCommandNoCANCoder restingWristPosition, ElevatorCloseCommand elevatorCloseCommand,IntakeCommand intakeCommand){
        addCommands(new SequentialCommandGroup(new ParallelCommandGroup(midPointCommand, elevatorCloseCommand),restingWristPosition),intakeCommand);
    }
}