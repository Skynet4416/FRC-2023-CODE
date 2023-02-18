// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.Motors;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftIntakeSparkMax = new CANSparkMax(Motors.kLeftIntakeCANID, Motors.kMotorType);
  private final CANSparkMax m_rightIntakeSparkMax = new CANSparkMax(Motors.kRightIntakeCANID, Motors.kMotorType);

  public IntakeSubsystem() {
    m_leftIntakeSparkMax.restoreFactoryDefaults();
    m_rightIntakeSparkMax.restoreFactoryDefaults();
    m_rightIntakeSparkMax.setInverted(true);
  }

  /**
   * Sets the speed of the intake motors.
   * <p>
   * 
   * @param percent The percent to set the intake motors to (Can be from -1 to 1.)
   * 
   */
  public void setPercentage(double percent) {
    System.out.println("Intake speed: " + percent + ".");
    m_leftIntakeSparkMax.set(percent);
    m_rightIntakeSparkMax.set(percent);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
  }
}