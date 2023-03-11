package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorCloseCommand extends CommandBase{
    private final ElevatorSubsystem m_elevator;
    public ElevatorCloseCommand(ElevatorSubsystem elevator){
        this.m_elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void initialize()
    {
        m_elevator.setPrecentage(0.2);
    }
    @Override
    public boolean isFinished(){
        return m_elevator.isClosed();
    }
    @Override
    public void end(boolean interrupted){
        m_elevator.setPrecentage(0);
    }
}
