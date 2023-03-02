package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class MovePrecentageCommand extends CommandBase{
    private final ArmSubsystem m_armSubsystem;
    private final double m_precentage;
    public MovePrecentageCommand(ArmSubsystem armSubsystem, double precentage){
        m_armSubsystem=  armSubsystem;
        m_precentage = precentage;
    }
    @Override
    public void initialize()
    {
        m_armSubsystem.setPrecentage(m_precentage);
    }
    @Override
    public void end(boolean interrupted)
    {
        m_armSubsystem.setPrecentage(0);
    }
}
