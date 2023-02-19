package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Wrist.Physical;
import frc.robot.subsystems.Wrist.WristSubsystem;

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
        m_subsystem.setAngle(angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.setVoltage(m_subsystem.calculate());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_subsystem.getWristAngleInDegrees()-angle) < Physical.kErrorTolarance;
    }
}
