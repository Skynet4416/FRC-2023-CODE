package frc.robot.commands.Arm;

import frc.robot.Constants.Arm.Physical;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem m_subsystem;
  private final double m_wantedHeight;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem subsystem,double wantedHeight) {
    m_subsystem = subsystem;
    m_wantedHeight = wantedHeight;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setArmHeightInMeters(m_wantedHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.pushVoltage();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getArmHeightInMeters() - m_wantedHeight) < Physical.kHeightThreasholdInMeters;
  }
}
