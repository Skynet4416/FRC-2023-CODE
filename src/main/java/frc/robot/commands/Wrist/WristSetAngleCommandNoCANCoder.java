package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Wrist.Physical;
import frc.robot.subsystems.Wrist.WristNoCANCoder;
import frc.robot.subsystems.Wrist.WristSubsystem;

public class WristSetAngleCommandNoCANCoder extends CommandBase {
    private final WristNoCANCoder m_subsystem;
    private double angle;

    public WristSetAngleCommandNoCANCoder(WristNoCANCoder wristSubsystem, double angle) {
        m_subsystem = wristSubsystem;
        this.angle = angle;

        addRequirements(wristSubsystem);
    }
    public WristNoCANCoder getWrist(){
        return m_subsystem;
    }
    public double getAngle(){
        return angle;
    }
    public WristSetAngleCommandNoCANCoder(WristSetAngleCommandNoCANCoder wristSetAngleCommandNoCANCoder){
        this(wristSetAngleCommandNoCANCoder.getWrist(), wristSetAngleCommandNoCANCoder.getAngle());
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
        return Math.abs(m_subsystem.getWristAngleInDegrees() - angle) < Physical.kErrorTolarance;
    }
}
