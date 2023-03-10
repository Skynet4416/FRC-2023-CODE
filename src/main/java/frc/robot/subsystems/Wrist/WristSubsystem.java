package frc.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Wrist.Encoders;
import frc.robot.Constants.Wrist.FeedForward;
import frc.robot.Constants.Wrist.Motors;
import frc.robot.Constants.Wrist.PID;
import frc.robot.Constants.Wrist.Physical;

public class WristSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_wristSparkMax = new WPI_TalonFX(Motors.kWristSparkMaxCANID);
    private final CANCoder m_CANCoder = new CANCoder(Encoders.kWristCANCoderCANID);
    private final PIDController m_PidController = new PIDController(PID.kP, PID.kI, PID.kD);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(FeedForward.kS, FeedForward.kG, FeedForward.kV,
            FeedForward.kA);
    private double m_lastRotationalVelocity = 0;
    private double m_lastTimeStamp = Timer.getFPGATimestamp();
    private double m_setpoint;

    public WristSubsystem() {

        m_CANCoder.configFactoryDefault(); // Is this the right thing to do? yes.
        m_wristSparkMax.configFactoryDefault();
        m_wristSparkMax.setNeutralMode(NeutralMode.Brake);
        m_wristSparkMax.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 0.5, 30));
        SmartDashboard.putNumber("Wrist P", PID.kP);
        m_wristSparkMax.setInverted(true);
        m_PidController.setSetpoint(getWristAngleInDegrees());

    }

    public double getAbsuloteAngleInDegrees() {
        return m_CANCoder.getAbsolutePosition();
    }

    public double getAbsuloteAngleInRadianes() {
        return Units.degreesToRadians(getAbsuloteAngleInDegrees());
    }

    public double getWristAngleInDegrees() {
        return getAbsuloteAngleInDegrees() - Encoders.kCANCoderZeroAbsAngle;
    }

    public double getWristAngleInRadians() {

        return Units.degreesToRadians(getWristAngleInDegrees());
    }

    public WPI_TalonFX getSpark() {
        return m_wristSparkMax;
    }

    public CANCoder getCANCoder() {
        return m_CANCoder;
    }

    public double getRoationalVelocity() {
        return m_CANCoder.getVelocity();
    }

    public double getRotationalVelocityInRadians() {
        return Units.degreesToRadians(getRoationalVelocity());
    }

    public void setVoltage(double voltage) {
        m_wristSparkMax.setVoltage(voltage);
    }

    public void setPrecentage(double precentage) {
        m_wristSparkMax.set(precentage);
    }

    public double calculate() {
        return m_PidController.calculate(getWristAngleInDegrees());
    }

    public void setAngle(double angle) {
        m_PidController.setSetpoint(angle);
        SmartDashboard.putNumber("Wrist Setpoint", angle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Absolute Angle In Degrees", getAbsuloteAngleInDegrees());
        SmartDashboard.putNumber("Wrist Angle In Degrees", getWristAngleInDegrees());
        SmartDashboard.putNumber("Wrist Rotational Velocity In Degrees Per Second", getRoationalVelocity());
        SmartDashboard.putNumber("Wrist Rotational Velocity In Radians Per Second", getRotationalVelocityInRadians());
        SmartDashboard.putNumber("Wrist Acceleration In Radians Per Second Squered",
                (getRotationalVelocityInRadians() - m_lastRotationalVelocity)
                        / (Timer.getFPGATimestamp() - m_lastTimeStamp));
        SmartDashboard.putNumber("Wirst Error", m_PidController.getPositionError());
        SmartDashboard.putNumber("Wrist Voltage", m_wristSparkMax.get() * RobotController.getBatteryVoltage());
        m_PidController.setP(SmartDashboard.getNumber("Wrist P", PID.kP));
        m_lastTimeStamp = Timer.getFPGATimestamp();
        m_lastRotationalVelocity = getRotationalVelocityInRadians();
    }
}
