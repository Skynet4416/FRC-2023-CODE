// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    m_leftIntakeSparkMax.setInverted(false);
    m_rightIntakeSparkMax.follow(m_leftIntakeSparkMax, FollowerType.PercentOutput);
    m_rightIntakeSparkMax.setInverted(InvertType.OpposeMaster);
    m_leftIntakeSparkMax.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 80, 1.0));
    m_leftIntakeSparkMax.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 40, 0.5));
    m_rightIntakeSparkMax.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 80, 1.0));
    m_rightIntakeSparkMax.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 40, 0.5));
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
    // m_leftIntakeSparkMax.set(ControlMode.Current, percent);
  }
  public void setAmprage(double amps) {
    // System.out.println("Intake speed: " + percent + ".");
    m_leftIntakeSparkMax.set(ControlMode.Current,amps);
    // m_leftIntakeSparkMax.set(ControlMode.Current, percent);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Stator Current", m_rightIntakeSparkMax.getStatorCurrent());
    SmartDashboard.putNumber("Supply Current", m_rightIntakeSparkMax.getSupplyCurrent());

  }
}