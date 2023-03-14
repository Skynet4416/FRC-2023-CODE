package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist.WristNoCANCoder;
import frc.robot.subsystems.Wrist.WristSubsystem;

public class WristKeepAngleNoCANCoderCommand extends CommandBase {
    private final WristNoCANCoder m_subsystem;

    public WristKeepAngleNoCANCoderCommand(WristNoCANCoder subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }
    public WristNoCANCoder getWristNoCANCoder(){
        return m_subsystem;
    }
    public WristKeepAngleNoCANCoderCommand(WristKeepAngleNoCANCoderCommand wristKeepAngleCommand){
        this(wristKeepAngleCommand.getWristNoCANCoder());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
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
        return false;
    }
}
