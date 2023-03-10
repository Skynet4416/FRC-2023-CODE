// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.Motors;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftIntakeSparkMax = new WPI_TalonFX(Motors.kLeftIntakeCANID);
  private final WPI_TalonFX m_rightIntakeSparkMax = new WPI_TalonFX(Motors.kRightIntakeCANID);

  public IntakeSubsystem() {
    m_leftIntakeSparkMax.configFactoryDefault();
    m_rightIntakeSparkMax.configFactoryDefault();
    m_leftIntakeSparkMax.setNeutralMode(NeutralMode.Brake);
    m_rightIntakeSparkMax.setNeutralMode(NeutralMode.Brake);
    m_leftIntakeSparkMax.setInverted(true);
    m_rightIntakeSparkMax.follow(m_leftIntakeSparkMax, FollowerType.PercentOutput);

    m_leftIntakeSparkMax.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,60,0.5));
    m_rightIntakeSparkMax.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,60,0.5));
  }

  /**
   * Sets the speed of the intake motors.
   * <p>
   * 
   * @param percent The percent to set the intake motors to (Can be from -1 to 1.)
   * 
   */
  public void setPercentage(double percent) {
    // System.out.println("Intake speed: " + percent + ".");
    m_leftIntakeSparkMax.set(percent);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
  }
}