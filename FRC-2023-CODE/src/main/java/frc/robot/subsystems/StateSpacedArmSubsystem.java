package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm.Physical;

public class StateSpacedArmSubsystem extends SubsystemBase {
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            Units.degreesToRadians(45),
            Units.degreesToRadians(90)); // Max arm speed and acceleration.
    private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
    // The plant holds a state-space model of our arm. This system has the following
    // properties:
    //
    // States: [position, velocity], in radians and radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [position], in radians.
    private final LinearSystem<N2, N1, N1> m_armPlant = LinearSystemId
            .createSingleJointedArmSystem(DCMotor.getFalcon500(1), Physical.kArmMomentOfInertia, Physical.kArmGearing);
    // The observer fuses our encoder data and voltage inputs to reject noise.

    private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            m_armPlant,
            VecBuilder.fill(0.015, 0.17), // How accurate we
            // think our model is, in radians and radians/sec.
            VecBuilder.fill(0.01), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading
            0.020);
    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
            m_armPlant,
            VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
            // Position and velocity error tolerances, in radians and radians per second.
            // Decrease
            // this to more heavily penalize state excursion, or make the controller behave
            // more
            // aggressively. In this example we weight position much more highly than
            // velocity, but this can be tuned to balance the two.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12
            // is a good starting point because that is the (approximate) maximum voltage of
            // a
            // battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using
                    // notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant
    // for easy control.
    private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(m_armPlant, m_controller, m_observer,
            12.0, 0.020);
    private final WPI_CANCoder m_armCANCoder = new WPI_CANCoder(0);
    private final WPI_TalonFX m_armTalonFX = new WPI_TalonFX(0);
    private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State(getAbsoluteAngle(),0);

    public StateSpacedArmSubsystem() {
        m_armTalonFX.configFactoryDefault();
        m_armCANCoder.configFactoryDefault();
        m_loop.reset(VecBuilder.fill(getArmAngle(), m_armCANCoder.getVelocity()));
    }

    public Vector<N2> getStateAsVector() {
        return VecBuilder.fill(getArmAngle(), getArmRoationalVelocity());
    }

    public TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getArmAngle(), getArmRoationalVelocity());
    }

    public double getArmRoationalVelocity() {
        return m_armCANCoder.getVelocity();
    }

    public double absoluteAngleToArmAngle(double absAngle) {
        return absAngle;
    }

    public double getArmAngle() {
        return absoluteAngleToArmAngle(getAbsoluteAngle());
    }

    public double getAbsoluteAngle() {
        return m_armCANCoder.getAbsolutePosition();
    }

    public void setVoltage(final double voltage) {
        m_armTalonFX.setVoltage(voltage);
    }

    public void setTargetState(TrapezoidProfile.State wantedState) {
        m_goalState = wantedState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Arm State", new double[] { getState().position, getState().velocity });
        SmartDashboard.putNumberArray("Arm Goal State", new Double[] { m_goalState.position, m_goalState.velocity });
    }

    public double heightToSystemAngle(double height) {
        return height;
    }

    public double systemAngleToAbsoluteAngle(double angle) {
        return angle;
    }

    public void setHeight(double height) {
        m_goalState = new TrapezoidProfile.State(systemAngleToAbsoluteAngle(heightToSystemAngle(height)), 0);
    }

    public void execute() {
        m_lastProfiledReference = (new TrapezoidProfile(m_constraints, m_goalState, m_lastProfiledReference))
                .calculate(0.020);
        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(getArmAngle()));

        // Update our LQR to generate new voltage commands and use the voltages to
        // predict the next state with out Kalman filter.
        m_loop.predict(0.020);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0);
        setVoltage(nextVoltage);
    }
    public double absoluteAngleToHeight(double angle)
    {
        return systemAngleToHeight(absoluteAngleToArmAngle(angle));
    }
    public double systemAngleToHeight(double angle){
        return angle;
    }
	public double getHeight() {
		return systemAngleToHeight(getArmAngle());
	}
}
