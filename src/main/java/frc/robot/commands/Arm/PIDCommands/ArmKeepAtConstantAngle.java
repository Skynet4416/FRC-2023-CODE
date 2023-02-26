package frc.robot.commands.Arm.PIDCommands;

import frc.robot.subsystems.Arm.PIDArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmKeepAtConstantAngle extends CommandBase {
    private final PIDArmSubsystem m_pidArmSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmKeepAtConstantAngle(PIDArmSubsystem pidArmSubsystem) {
        m_pidArmSubsystem = pidArmSubsystem;

        addRequirements(m_pidArmSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_pidArmSubsystem.setVoltage(m_pidArmSubsystem.calculateVoltage());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_pidArmSubsystem.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
