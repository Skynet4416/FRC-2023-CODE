package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import frc.robot.Constants.Arm.MotionMagicPID;

public class MotionMagicSubsystem extends ArmSubsystem{

    public MotionMagicSubsystem()
    {
        super();
        m_armFalcon.config_kF(0, MotionMagicPID.kFF);
        m_armFalcon.config_kP(0, MotionMagicPID.kP);
        m_armFalcon.config_kD(0, MotionMagicPID.kD);
        m_armFalcon.config_kI(0, MotionMagicPID.kI);
        
        
        
    }
    public double getFeedForward()
    {
        return Math.cos(getArmAngleinRadians()) * MotionMagicPID.kG; 
    }
    @Override
    public void setAngleInDegrees(double degrees) {
        m_armFalcon.set(ControlMode.MotionMagic, degrees, DemandType.ArbitraryFeedForward, getFeedForward());
    }
    
}
