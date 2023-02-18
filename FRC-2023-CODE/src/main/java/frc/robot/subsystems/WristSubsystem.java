package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;
import frc.robot.Constants.Wrist.Encoders;
import frc.robot.Constants.Wrist.FeedForward;
import frc.robot.Constants.Wrist.Motors;
import frc.robot.Constants.Wrist.PID;
import frc.robot.Constants.Wrist.Physical;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax m_wristSparkMax = new CANSparkMax(Motors.kWristCANID, MotorType.kBrushless);
    private CANCoder m_CANCoder = new CANCoder(Encoders.kWristCANID);
    private ProfiledPIDController m_PidController = new ProfiledPIDController(PID.kP,PID.kI,PID.kD,new Constraints(Physical.kMaxVelcoityRadiansPerSecond,Physical.kMaxAccelerationRadiansPerSecondSquered));
    private ArmFeedforward armFeedforward = new ArmFeedforward(FeedForward.kS, FeedForward.kG, FeedForward.kV,FeedForward.kA);
    public WristSubsystem() {
        m_CANCoder.configFactoryDefault(); // Is this the right thing to do? yes.
        m_wristSparkMax.restoreFactoryDefaults();
        
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
    
        return Units.degreesToRadians(getWristAngleInDegrees());}
    public CANSparkMax getSpark(){
        return m_wristSparkMax;
    }
    
    public CANCoder getCANCoder(){
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
    public double calculate(){
        return m_PidController.calculate(getWristAngleInRadians()) + armFeedforward.calculate(getWristAngleInRadians(), getRotationalVelocityInRadians());
    }
    public void setAngle(double angle){
        m_PidController.setGoal(new State(angle, 0));
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Wrist Absolute Angle", getAbsuloteAngleInDegrees());
        SmartDashboard.putNumber("Wrist Angle", getWristAngleInDegrees());
        SmartDashboard.putNumber("Wrist Rotational Velocity", getRoationalVelocity());

    }
}
