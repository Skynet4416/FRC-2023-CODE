package frc.robot.commands.drive.Auto;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.PIDAngular;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class TurnToConstantAngle extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final PIDController m_PID;
    private double angle;

    public TurnToConstantAngle(DriveSubsystem driveSubsystem, double angle) {
        m_driveSubsystem = driveSubsystem;
        m_PID = m_driveSubsystem.getRotationalPIDController();
        addRequirements(m_driveSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        m_PID.setSetpoint(Units.degreesToRadians(angle) + m_driveSubsystem.getHeading().getRadians());
    }

    @Override
    public void execute() {
        double voltage = m_PID.calculate(m_driveSubsystem.getHeading().getRadians());
        SmartDashboard.putNumber("Drive Angular Voltage" , voltage);
        m_driveSubsystem.setVoltage(voltage, -voltage);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_PID.getPositionError()) < Units.degreesToRadians(PIDAngular.kAngleThreashold);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.setVoltage(0, 0);
    }

}
