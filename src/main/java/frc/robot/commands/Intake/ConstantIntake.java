package frc.robot.commands.Intake;

import com.fasterxml.jackson.databind.node.IntNode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class ConstantIntake extends CommandBase{
    private final IntakeSubsystem intakeSubsystem;
    public ConstantIntake(IntakeSubsystem intakeSubsystem)
    {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void execute()
    {
        if(Globals.hasGamePice)
        {
            intakeSubsystem.setPercentage(0.1);
        }
    }
}
