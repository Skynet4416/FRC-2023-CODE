package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm.Encoders;
import frc.robot.Constants.Arm.Motors;
import frc.robot.Constants.Arm.Physical;

public abstract class ArmSubsystem extends SubsystemBase {
    protected CANSparkMax m_armSparkMax = new CANSparkMax(Motors.kArmCANID, MotorType.kBrushless);
    protected CANCoder m_CANCoder = new CANCoder(Encoders.kCANCoderID);
    private double m_lastRotationalVelocity = 0;
    private double m_lastTimeStamp = Timer.getFPGATimestamp();

    public ArmSubsystem() {
        m_armSparkMax.restoreFactoryDefaults();
        m_CANCoder.configFactoryDefault();
        m_armSparkMax.setIdleMode(IdleMode.kBrake);
        m_armSparkMax.enableVoltageCompensation(12);
        m_armSparkMax.setSmartCurrentLimit(30);
    }

    public double unitsToDegrees(double units) {
        return (double) units * 360 / 4096;
    }

    public double degreesToUnits(double degrees) {
        return (double) (degrees * 4096 / 360);
    }

    public double getAbsuloteAngleInDegrees() {
        return m_CANCoder.getAbsolutePosition();
    }

    public double degreesToHeightInMeters(double degrees) {
        return Math.sin(Units.degreesToRadians(degrees)) * Physical.kArmLength + Physical.kArmHeight;
    }

    public double getArmAngleinRadians() {
        return Units.degreesToRadians(getArmAngleInDegrees());
    }

    public double getArmRoationalVelocityInRadiansPerSecond() {
        return Units.degreesToRadians(getArmRoationalVelocity());
    }

    public double heightInMetersToDegrees(double height) {
        return Units.radiansToDegrees(Math.asin((height - Physical.kArmHeight) / Physical.kArmLength));
    }

    public double getArmAngleInDegrees() // 0 angle = parallel to the ground forward
    {
        return getAbsuloteAngleInDegrees() - Encoders.kCANCoderZeroAngle;
    }

    public double getArmHeightInMeters() {
        return degreesToHeightInMeters(getArmAngleInDegrees());
    }

    public double getArmRoationalVelocity() {
        return m_CANCoder.getVelocity();
    }

    public void setVoltage(double voltage) {
        m_armSparkMax.setVoltage(voltage);
    }

    public void setPrecentage(double precentage) {
        m_armSparkMax.set(precentage);
    }

    public abstract void setAngleInDegrees(double degrees);

    public void setHeight(double height) {
        setAngleInDegrees(heightInMetersToDegrees(height));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Absolute Angle In Degrees", getAbsuloteAngleInDegrees());
        SmartDashboard.putNumber("Arm Angle In Degrees", getArmAngleInDegrees());
        SmartDashboard.putNumber("Arm Rotational Velocity In Degrees Per Second", getArmRoationalVelocity());
        SmartDashboard.putNumber("Arm Rotational Velocity In Radians Per Second",
                getArmRoationalVelocityInRadiansPerSecond());
        SmartDashboard.putNumber("Arm Acceleration In Radians Per Second Squered",
                (getArmRoationalVelocityInRadiansPerSecond() - m_lastRotationalVelocity)
                        / (Timer.getFPGATimestamp() - m_lastTimeStamp));
        SmartDashboard.putNumber("Arm Voltage", m_armSparkMax.get() * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Arm Current", m_armSparkMax.getOutputCurrent());
        m_lastTimeStamp = Timer.getFPGATimestamp();
        m_lastRotationalVelocity = getArmRoationalVelocityInRadiansPerSecond();
    }

    public void lockArm() {
        m_armSparkMax.setIdleMode(IdleMode.kBrake);
        System.out.println("Arm Locked");
    }

    public void unlockArm() {
        m_armSparkMax.setIdleMode(IdleMode.kCoast);
        System.out.println("Arm Unlocked");

    }

}
