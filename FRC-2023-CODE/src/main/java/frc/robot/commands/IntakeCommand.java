// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem m_subsystem;
  private final DoubleSupplier m_right;
  private final DoubleSupplier m_left;
  
  public IntakeCommand(IntakeSubsystem subsystem, DoubleSupplier m_left, DoubleSupplier m_right) {
    this.m_subsystem = subsystem;
    this.m_left = m_left;
    this.m_right = m_right;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double triggerVal = m_right.getAsDouble() - m_left.getAsDouble();
    m_subsystem.setPercentage(triggerVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPercentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
