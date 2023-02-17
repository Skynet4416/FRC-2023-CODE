package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm.Motors;

public abstract class ArmSubsystem extends SubsystemBase{
    private WPI_TalonFX m_armFalcon = new WPI_TalonFX(Motors.kArmCANID);
    public ArmSubsystem()
    {
        m_armFalcon.configFactoryDefault();
        m_armFalcon.setNeutralMode(NeutralMode.Brake);
        m_armFalcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        m_armFalcon.configVoltageCompSaturation(12);
        m_armFalcon.configSupplyCurrentLimit()
    }
}
