package frc.robot.commands.Arm.StateSpaceCommands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.StateSpacedArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class KeepArmAtStateCommand extends CommandBase {
    private final StateSpacedArmSubsystem m_stateSpaceArmSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public KeepArmAtStateCommand(StateSpacedArmSubsystem stateSpaceArmSubsystem) {
        m_stateSpaceArmSubsystem = stateSpaceArmSubsystem;

        addRequirements(m_stateSpaceArmSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_stateSpaceArmSubsystem.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_stateSpaceArmSubsystem.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
