package frc.robot.commands.Arm.StateSpaceCommands;

import frc.robot.Constants.Arm.Physical;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.StateSpacedArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToConstantHeightCommand extends CommandBase {
    private final StateSpacedArmSubsystem m_spaceStateArmSubsystem;

    private final double m_wantedHeight;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmToConstantHeightCommand(StateSpacedArmSubsystem subsystem,double wantedHeight) {
        m_spaceStateArmSubsystem = subsystem;
        m_wantedHeight = wantedHeight;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_spaceStateArmSubsystem.setHeight(m_wantedHeight);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_spaceStateArmSubsystem.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_spaceStateArmSubsystem.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_wantedHeight - m_spaceStateArmSubsystem.getArmHeightInMeters())< Physical.kHeightThreasholdInMeters;
    }
}
