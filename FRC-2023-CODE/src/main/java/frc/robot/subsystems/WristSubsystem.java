package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist.Encoders;
import frc.robot.Constants.Wrist.Motors;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax m_wristSparkMax = new CANSparkMax(Motors.kWristCANID, MotorType.kBrushless);
    private CANCoder m_CANCoder = new CANCoder(Encoders.kWristCANID);

    public WristSubsystem() {
        m_CANCoder.configFactoryDefault(); // Is this the right thing to do?
        // Not resetting spark max because it isn't reset in IntakeSubsystem.
    }

    public double getAbsuloteAngleInDegrees() {
        return m_CANCoder.getAbsolutePosition();
    }

    public double getRoationalVelocity() {
        return m_CANCoder.getVelocity();
    }

    public void setVoltage(double voltage) {
        m_wristSparkMax.setVoltage(voltage);
    }

    public void setPrecentage(double precentage) {
        m_wristSparkMax.set(precentage);
    }
}
