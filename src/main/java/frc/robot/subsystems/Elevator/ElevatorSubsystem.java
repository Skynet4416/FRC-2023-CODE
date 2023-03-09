package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.LimitSwitch;
import frc.robot.Constants.Elevator.Motors;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftElevatorCANSparMax = new CANSparkMax(Motors.kLeftSparkMaxCANID,
            MotorType.kBrushless);
    private final CANSparkMax m_rightElevatorCANSparMax = new CANSparkMax(Motors.kRightSparkMaxCANID,
            MotorType.kBrushless);
    private final DigitalInput m_closedLimitSwitch = new DigitalInput(LimitSwitch.kClosedLimitSwitchPort);
    private final DigitalInput m_openedLimitSwitch = new DigitalInput(LimitSwitch.kOpendLimitSwitchPort);

    public ElevatorSubsystem() {
        m_leftElevatorCANSparMax.restoreFactoryDefaults();
        m_rightElevatorCANSparMax.restoreFactoryDefaults();
        m_rightElevatorCANSparMax.follow(m_leftElevatorCANSparMax, true);
        m_leftElevatorCANSparMax.setIdleMode(IdleMode.kBrake);
        m_rightElevatorCANSparMax.setIdleMode(IdleMode.kBrake);
        m_leftElevatorCANSparMax.setSmartCurrentLimit(30);
        m_rightElevatorCANSparMax.setSmartCurrentLimit(30);
        m_leftElevatorCANSparMax.enableVoltageCompensation(12);
        m_rightElevatorCANSparMax.enableVoltageCompensation(12);
    }

    public void setPrecentage(double precentage) {
        m_leftElevatorCANSparMax.set(precentage);
    }

    @Override
    public void periodic() {
        if (getLimits()) {
            m_leftElevatorCANSparMax.set(0);
            m_rightElevatorCANSparMax.set(0);
        }
        SmartDashboard.putNumber("Elevator Precentage", m_leftElevatorCANSparMax.get());
    }

    public boolean getLimits() {
        return isClosed() || isOpened();
    }

    public boolean isClosed() {
        return m_closedLimitSwitch.get();
    }

    public boolean isOpened() {
        return m_openedLimitSwitch.get();
    }

    public void lockElevator() {
        m_leftElevatorCANSparMax.setIdleMode(IdleMode.kBrake);
        m_rightElevatorCANSparMax.setIdleMode(IdleMode.kBrake);
    }

    public void unlockElevator() {

        m_leftElevatorCANSparMax.setIdleMode(IdleMode.kCoast);
        m_rightElevatorCANSparMax.setIdleMode(IdleMode.kCoast);
    }

}
