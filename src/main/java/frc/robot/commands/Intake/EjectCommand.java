// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class EjectCommand extends CommandBase {
  private final IntakeSubsystem m_subsystem;
  private final double Percentage;

  public EjectCommand(IntakeSubsystem subsystem, double Percentage) {
    this.m_subsystem = subsystem;
    this.Percentage = Percentage;
    addRequirements(subsystem);
  }
  public IntakeSubsystem getIntake(){
    return m_subsystem;
  }
  public double getPrecentage(){
    return Percentage;
  }
  public EjectCommand(EjectCommand ejectCommand){
    this(ejectCommand.getIntake(),ejectCommand.getPrecentage());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPercentage(-this.Percentage);
    Globals.hasGamePice = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
