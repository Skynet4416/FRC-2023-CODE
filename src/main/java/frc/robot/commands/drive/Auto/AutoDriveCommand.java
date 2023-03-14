// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.DrivingState;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/** An example command that uses an example subsystem. */
public class AutoDriveCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveSubsystem m_system;
    private final double m_left;
    private final double m_right;
    private final DrivingState m_state;
    private final VisionSubsystem m_VisionSubsystem;

    public AutoDriveCommand(DriveSubsystem m_system, DrivingState m_state, double m_left, double m_right,
            VisionSubsystem visionSubsystem) {
        this.m_system = m_system;
        this.m_state = m_state;
        this.m_left = m_left;
        this.m_right = m_right;
        this.m_VisionSubsystem = visionSubsystem;

        addRequirements(m_system, visionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (m_state) {
            case TANK:
                m_system.setTankDrive(m_left, m_right);

            case ARCADE:
                m_system.setArcadeDrive(m_left, m_right);
            case CURVATURE:
                m_system.setCurvatureDrive(m_left,m_right , true);

            default:
                // INSERT ERROR
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_system.setVoltage(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
