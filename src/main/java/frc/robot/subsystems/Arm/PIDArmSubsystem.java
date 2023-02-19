package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Arm.Physical;
import frc.robot.Constants.Arm.PoistionPID.ArbitraryFeedForward;
import frc.robot.Constants.Arm.PoistionPID.PID;

public class PIDArmSubsystem extends ArmSubsystem {
    private ProfiledPIDController pidController = new ProfiledPIDController(PID.kP, PID.kI, PID.kD, new Constraints(
            Physical.kArmMaxVelocityRadiansPerSecond, Physical.kArmMaxAccelerationRadiansPerSecondSquered));
    private ArmFeedforward feedForward = new ArmFeedforward(ArbitraryFeedForward.kS, ArbitraryFeedForward.kG,
            ArbitraryFeedForward.kV, ArbitraryFeedForward.kA);

    public PIDArmSubsystem() {
        super();
    }

    @Override
    public void setAngleInDegrees(double degrees) {
        super.periodic();
        pidController.setGoal(Units.degreesToRadians(degrees));
    }

    public double calculateVoltage() {
        return pidController.calculate(getArmAngleinRadians(), getArmRoationalVelocityInRadiansPerSecond())
                + feedForward.calculate(getArmAngleinRadians(), getArmRoationalVelocityInRadiansPerSecond());
    }
}
