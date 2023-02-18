package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm.Encoders;
import frc.robot.Constants.Arm.Motors;
import frc.robot.Constants.Arm.Physical;

public abstract class ArmSubsystem extends SubsystemBase {
    protected WPI_TalonFX m_armFalcon = new WPI_TalonFX(Motors.kArmCANID);
    protected CANCoder m_CANCoder = new CANCoder(Encoders.kCANCoderID);
    public ArmSubsystem() {
        m_armFalcon.configFactoryDefault();
        m_CANCoder.configFactoryDefault();
        m_armFalcon.setNeutralMode(NeutralMode.Brake);
        m_armFalcon.configRemoteFeedbackFilter(m_CANCoder, 0);
        m_armFalcon.configVoltageCompSaturation(12);
        m_armFalcon.configClosedLoopPeakOutput(0, 1);
        m_armFalcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, Units.millisecondsToSeconds(150)));
    }

    public double unitsToDegrees(double units) {
        return (double) units * 360 / 4096;
    }

    public double degreesToUnits(double degrees) {
        return (double) (degrees * 4096 / 360);
    }
    public double getAbsuloteAngleInDegrees()
    {
        return m_CANCoder.getAbsolutePosition();
    }
    public double degreesToHeightInMeters(double degrees)
    {
        return Math.sin(Units.degreesToRadians(degrees)) * Physical.kArmLength + Physical.kArmHeight;
    }
    public double getArmAngleinRadians()
    {
        return Units.degreesToRadians(getArmAngleInDegrees());
    }
    public double getArmRoationalVelocityInRadiansPerSecond()
    {
        return Units.degreesToRadians(getArmRoationalVelocity());
    }
    public double heightInMetersToDegrees(double height)
    {
        return Units.radiansToDegrees(Math.asin((height-Physical.kArmHeight)/Physical.kArmLength));
    }

    public double getArmAngleInDegrees() // 0 angle = parallel to the ground forward
    {
        return getAbsuloteAngleInDegrees() - Encoders.kCANCoderZeroAngle;
    }
    public double getArmHeightInMeters()
    {
        return degreesToHeightInMeters(getArmAngleInDegrees());
    }
    public double getArmRoationalVelocity()
    {
        return m_CANCoder.getVelocity();
    }
    public void setVoltage(double voltage)
    {
        m_armFalcon.setVoltage(voltage);
    }
    public void setPrecentage(double precentage)
    {
        m_armFalcon.set(ControlMode.PercentOutput, precentage);
    }
    public abstract void setAngleInDegrees(double degrees);
    public void setHeight(double height)
    {
        setAngleInDegrees(heightInMetersToDegrees(height));
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Absolute Angle", getAbsuloteAngleInDegrees());
        SmartDashboard.putNumber("Arm Angle", getAbsuloteAngleInDegrees());
        SmartDashboard.putNumber("Arm Height", getArmHeightInMeters());
        SmartDashboard.putNumber("Arm Velocity", getArmRoationalVelocity());


    }
    

}
