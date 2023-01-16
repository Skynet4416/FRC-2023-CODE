package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Motors;
import frc.robot.Constants.Drive.Physical;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftForwardSparkMax = new CANSparkMax(Motors.kLeftForwardCANID, Motors.kMotorType);
    private final CANSparkMax m_rightForwardSparkMax = new CANSparkMax(Motors.kRightForwardCANID, Motors.kMotorType);
    private final CANSparkMax m_leftBackwardSparkMax = new CANSparkMax(Motors.kLeftBackwardCANID, Motors.kMotorType);
    private final CANSparkMax m_rightBackwardSparkMax = new CANSparkMax(Motors.kRightBackwardCANID, Motors.kMotorType);
    private final MotorControllerGroup m_leftControllerGroup = new MotorControllerGroup(m_leftBackwardSparkMax,
            m_leftForwardSparkMax);
    private final MotorControllerGroup m_rightControllerGroup = new MotorControllerGroup(m_rightBackwardSparkMax,
            m_rightForwardSparkMax);
    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftControllerGroup,
            m_rightControllerGroup);
    private final RelativeEncoder m_leftForwardRelativeEncoder = m_leftForwardSparkMax.getAlternateEncoder(1);
    private final RelativeEncoder m_rightForwardRelativeEncoder = m_rightForwardSparkMax.getAlternateEncoder(1);
    private final RelativeEncoder m_leftBackwardRelativeEncoder = m_leftBackwardSparkMax.getAlternateEncoder(1);
    private final RelativeEncoder m_rightBackwardRelativeEncoder = m_rightBackwardSparkMax.getAlternateEncoder(1);
    private final AHRS m_navx = new AHRS();
    private final DifferentialDriveKinematics m_differentialDriveKinematics = new DifferentialDriveKinematics(
            Physical.kDistanceBetweenLeftAndRightWheelsInMeters);

    private final DifferentialDrivePoseEstimator m_differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(
            m_differentialDriveKinematics, getHeading(), getLeftDistance(), getRightDistance(),
            Physical.kDeafultPosition, Physical.kStateStdDevs, Physical.kVisionMeasurementStdDevs);
    private final Field2d m_field2d = new Field2d();
    private final DifferentialDrivetrainSim m_differentialDrivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDoubleNEOPerSide,
            KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    private final SimDouble m_simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

    // The left-side drive encoder
    private final Encoder m_leftCheatingEncoder;

    // The right-side drive encoder
    private final Encoder m_rightCheatingEncoder;
    private final EncoderSim m_leftSimulatedEncoder;
    private final EncoderSim m_rightSimulatedEncoder;

    public DriveSubsystem() {
        restoreFactoryDefaults();
        setIdleMode(IdleMode.kCoast); // being able to move the robot
        resetEncoders();
        m_leftControllerGroup.setInverted(true);
        //
        m_leftBackwardSparkMax.enableVoltageCompensation(12);
        m_leftForwardSparkMax.enableVoltageCompensation(12);
        m_rightBackwardSparkMax.enableVoltageCompensation(12);
        m_rightForwardSparkMax.enableVoltageCompensation(12);
        //
        m_leftBackwardSparkMax.setSmartCurrentLimit(30);
        m_leftForwardSparkMax.setSmartCurrentLimit(30);
        m_rightBackwardSparkMax.setSmartCurrentLimit(30);
        m_rightForwardSparkMax.setSmartCurrentLimit(30);
        if (RobotBase.isSimulation()) {

            m_rightCheatingEncoder = new Encoder(
                    Drive.CheatedEncodersPorts.kRightEncoderPorts[0],
                    Drive.CheatedEncodersPorts.kRightEncoderPorts[1],
                    Drive.CheatedEncodersPorts.kRightEncoderReversed);
            m_leftCheatingEncoder = new Encoder(
                    Drive.CheatedEncodersPorts.kLeftEncoderPorts[0],
                    Drive.CheatedEncodersPorts.kLeftEncoderPorts[1],
                    Drive.CheatedEncodersPorts.kLeftEncoderReversed);
            m_leftCheatingEncoder.setDistancePerPulse(Drive.CheatedEncodersPorts.kEncoderDistancePerPulse);
            m_rightCheatingEncoder.setDistancePerPulse(Drive.CheatedEncodersPorts.kEncoderDistancePerPulse);
            m_leftSimulatedEncoder = new EncoderSim(m_leftCheatingEncoder);
            m_rightSimulatedEncoder = new EncoderSim(m_rightCheatingEncoder);

        } else {
            m_rightSimulatedEncoder = null;
            m_leftSimulatedEncoder = null;
            m_rightCheatingEncoder = null;
            m_leftCheatingEncoder = null;
        }
        m_navx.calibrate();
    }

    public double getLeftDistance() {
        return (m_leftBackwardRelativeEncoder.getPosition()
                + m_leftForwardRelativeEncoder.getPosition()) / 2;
    }

    public double getRightDistance() {
        return (m_rightBackwardRelativeEncoder.getPosition()
                + m_rightForwardRelativeEncoder.getPosition()) / 2;
    }

    public double getLeftVelocity() {
        return (m_leftBackwardRelativeEncoder.getVelocity()
                + m_leftForwardRelativeEncoder.getVelocity()) / 2;
    }

    public double getRightVelocity() {
        return (m_rightBackwardRelativeEncoder.getVelocity()
                + m_rightForwardRelativeEncoder.getVelocity()) / 2;
    }

    public double getCheatedLeftDistance() {
        return m_leftCheatingEncoder.getDistance();
    }

    public double getCheatedRightDistance() {
        return m_rightCheatingEncoder.getDistance();
    }

    public double getCheatedLeftVelocity() {
        return m_leftCheatingEncoder.getRate();
    }

    public double getCheatedRightVelocity() {
        return m_rightCheatingEncoder.getRate();
    }

    public void setFactoConvertionFactor() {
        // convert units to meters
        m_leftBackwardRelativeEncoder.setPositionConversionFactor(Physical.kUnitsToMeters);
        m_rightBackwardRelativeEncoder.setPositionConversionFactor(Physical.kUnitsToMeters);
        m_leftForwardRelativeEncoder.setPositionConversionFactor(Physical.kUnitsToMeters);
        m_rightForwardRelativeEncoder.setPositionConversionFactor(Physical.kUnitsToMeters);
        // convert units/minute to meter/second
        m_leftBackwardRelativeEncoder.setVelocityConversionFactor(Physical.kUnitsToMeters);
        m_rightBackwardRelativeEncoder.setVelocityConversionFactor(Physical.kUnitsToMeters);
        m_leftForwardRelativeEncoder.setVelocityConversionFactor(Physical.kUnitsToMeters);
        m_rightForwardRelativeEncoder.setVelocityConversionFactor(Physical.kUnitsToMeters);
    }

    public void resetEncoders() {
        m_leftBackwardRelativeEncoder.setPosition(0);
        m_rightBackwardRelativeEncoder.setPosition(0);
        m_rightForwardRelativeEncoder.setPosition(0);
        m_leftForwardRelativeEncoder.setPosition(0);

        setFactoConvertionFactor();
    }

    public void restoreFactoryDefaults() {
        m_leftBackwardSparkMax.restoreFactoryDefaults();
        m_rightBackwardSparkMax.restoreFactoryDefaults();
        m_leftForwardSparkMax.restoreFactoryDefaults();
        m_rightForwardSparkMax.restoreFactoryDefaults();
    }

    public void setIdleMode(final IdleMode idleMode) {
        m_leftBackwardSparkMax.setIdleMode(idleMode);
        m_rightBackwardSparkMax.setIdleMode(idleMode);
        m_leftForwardSparkMax.setIdleMode(idleMode);
        m_rightForwardSparkMax.setIdleMode(idleMode);

    }

    public void setCurvatureDrive(final double xSpeed, final double zSpeed, final boolean turnInPlace) {

        m_differentialDrive.curvatureDrive(xSpeed, zSpeed, turnInPlace);
    }

    public void setTankDrive(final double leftSpeed, final double rightSpeed) {
        m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void setVoltage(final double leftVoltage, final double rightVoltage) {
        m_leftControllerGroup.setVoltage(leftVoltage);
        m_rightControllerGroup.setVoltage(rightVoltage);
        m_differentialDrive.feed();
    }

    public void setArcadeDrive(final double xSpeed, final double zSpeed) {
        m_differentialDrive.arcadeDrive(xSpeed, zSpeed);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_navx.getAngle());
    }

    public void addVisionMessurement(final Pose2d visionMessurement, final double delayInSeconds) {
        m_differentialDrivePoseEstimator.addVisionMeasurement(visionMessurement,
                Timer.getFPGATimestamp() - delayInSeconds);
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            m_differentialDrivePoseEstimator.update(
                    Rotation2d.fromDegrees(m_simAngle.get()), m_rightSimulatedEncoder.getDistance(),
                    m_leftSimulatedEncoder.getDistance());
        } else {

            m_differentialDrivePoseEstimator.update(getHeading(),
                    getCheatedLeftDistance(),
                    getCheatedRightDistance());
        }
        m_field2d.setRobotPose(m_differentialDrivePoseEstimator.getEstimatedPosition());
        updateSmartDashboard();
    }
    public void updateSmartDashboard()
    {
        SmartDashboard.putData(m_field2d);
    }

    @Override
    public void simulationPeriodic() {
        m_differentialDrivetrainSim.setInputs(
                m_leftControllerGroup.get() * RobotController.getBatteryVoltage(),
                m_rightControllerGroup.get() * RobotController.getBatteryVoltage());
        m_differentialDrivetrainSim.update(0.020);

        // is m_drivetrainSimulator the refrence of ur m_differentialDrivetrainSim?
        m_leftSimulatedEncoder.setDistance(m_differentialDrivetrainSim.getLeftPositionMeters());
        m_leftSimulatedEncoder.setRate(m_differentialDrivetrainSim.getLeftVelocityMetersPerSecond());
        m_rightSimulatedEncoder.setDistance(m_differentialDrivetrainSim.getRightPositionMeters());
        m_rightSimulatedEncoder.setRate(m_differentialDrivetrainSim.getRightVelocityMetersPerSecond());
        m_simAngle.set(m_differentialDrivetrainSim.getHeading().getDegrees());
    }
}
