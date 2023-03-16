package frc.robot.commands.drive.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive.PIDAngular;
import frc.robot.subsystems.Drive.DriveSubsystem;

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
        double precentage = m_PID.calculate(m_driveSubsystem.getHeading().getRadians());
        SmartDashboard.putNumber("Drive Angular Precentage" , precentage);
        m_driveSubsystem.setArcadeDrive(0, precentage);
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
