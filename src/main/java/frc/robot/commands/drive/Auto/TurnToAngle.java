package frc.robot.commands.drive.Auto;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.PIDAngular;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class TurnToAngle extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final ProfiledPIDController m_PID;
    public TurnToAngle(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_PID = m_driveSubsystem.getRotationalPIDController();
        addRequirements(m_driveSubsystem);
        addRequirements(m_visionSubsystem);
    }

    @Override
    public void initialize() {
        PhotonTrackedTarget target = m_visionSubsystem.getVisionTarget();
        if (target == null) {
            end(true);
            return;
        }
        double angle = target.getYaw();
        m_PID.setGoal(Units.degreesToRadians(angle)+m_driveSubsystem.getHeading().getRadians());        
    }
    @Override
    public void execute(){
        double voltage = m_PID.calculate(m_driveSubsystem.getHeading().getRadians());
        m_driveSubsystem.setVoltage(voltage, -voltage);
    }
    @Override
    public boolean isFinished()
    {
        PhotonTrackedTarget target = m_visionSubsystem.getVisionTarget();
        if (target == null) {
            end(true);
            return true;
        }
        double angle = target.getYaw();
        return Math.abs(angle)<PIDAngular.kAngleThreashold;
    }
    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.setVoltage(0, 0);
    }

}
