package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm.Encoders;
import frc.robot.Constants.Arm.Motors;

public class ArmSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_motor = new WPI_TalonFX(Motors.kArmCANID);
  private final CANCoder m_cancoder = new CANCoder(Encoders.kCANCoderID);
  private final PIDController m_armPIDController = new PIDController(0.1, 0, 0);
  private final ArmFeedforward m_armFeedForward = new ArmFeedforward(0, 0, 0);

  public ArmSubsystem() {
    m_cancoder.configFactoryDefault();
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Brake);    
  }

  /**
   * Gets the angle of the arm.
   * 
   * @return the degree of the arm (Degrees)
   */
  public double getArmAngleInDegrees() {
    return m_cancoder.getAbsolutePosition();
  }

  /**
   * Gets the height of the arm in degrees
   * @param height (Meters)
   * @return
   */
  public double heightInMetersToDegrees(double height)
  {
    // TODO: Calculate the height of the arm in degrees (dont know the default degree of the arm yet) 
    return 0;
  }
  
  /**
   * Sets the voltage of the arm motor.
   * @param voltage (double)
   */
  public void setVoltage(double voltage)
  {
    m_motor.setVoltage(voltage);
  }
  
  public double getArmSpeedInRadiansPerSecond()
  {
    return Units.degreesToRadians(m_cancoder.getVelocity());
  }
  
  /**
   * Gets the height of the arm (Meters)
   * @param ArmHeightInMeters
   */
  public void setArmHeightInMeters(double ArmHeightInMeters)
  {
    m_armPIDController.setSetpoint(Units.degreesToRadians(heightInMetersToDegrees(ArmHeightInMeters)));
  }
  public void pushVoltage()
  {
    setVoltage(MathUtil.clamp(m_armPIDController.calculate(getArmAngleInRadians()) + m_armFeedForward.calculate(getArmAngleInRadians(), getArmSpeedInRadiansPerSecond()),-12,12));
  }



  /**
   * Gets the height of the arm by translating the angle to a height.
   * 
   * @param angle (double)
   * @return height (double)
   */
  public double getArmHeightInMetersFromAngle(double angle) {
    // TODO: Calculate the height of the arm from the angle (dont know the default degree of the arm yet)
    return 0;
  }
  public double getArmHeightInMeters()
  {
    return getArmHeightInMetersFromAngle(getArmAngleInDegrees());
  }

  public double getArmAngleInRadians()
  {
    return Units.degreesToRadians(getArmAngleInDegrees());
  }

  public void keepHeight()
  {
    m_armFeedForward.calculate(getArmAngleInRadians(), 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle (Degrees)", getArmAngleInDegrees());
    SmartDashboard.putNumber("Arm Height (Meters)", getArmHeightInMeters());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
