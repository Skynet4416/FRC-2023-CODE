package frc.robot.commands.Arm.PIDCommands;

import frc.robot.Constants.Arm.Physical;
import frc.robot.subsystems.Arm.PIDArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToConstantAngleCommand extends CommandBase {
    private final PIDArmSubsystem m_PidArmSubsystem;

    private final double m_wantedAngle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmToConstantAngleCommand(PIDArmSubsystem subsystem, double wantedHeight) {
        m_PidArmSubsystem = subsystem;
        m_wantedAngle = wantedHeight; // why, this is just dumb way to do it

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_PidArmSubsystem.setAngleInDegrees(m_wantedAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_PidArmSubsystem.setVoltage(m_PidArmSubsystem.calculateVoltage());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_PidArmSubsystem.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_wantedAngle - m_PidArmSubsystem.getArmAngleInDegrees()) < Physical.kArmAngleThreasholdInDegrees;
    }
}
