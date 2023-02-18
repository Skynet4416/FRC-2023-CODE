package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristSetAngleCommand extends CommandBase {
    private final WristSubsystem m_subsystem;
    private double angle;

    public WristSetAngleCommand(WristSubsystem subsystem, double angle) {
        m_subsystem = subsystem;
        this.angle = angle;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // TODO: Control wrist using PID.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
