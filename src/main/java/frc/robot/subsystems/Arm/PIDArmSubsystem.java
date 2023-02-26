package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Arm.Physical;
import frc.robot.Constants.Arm.PoistionPID.ArbitraryFeedForward;
import frc.robot.Constants.Arm.PoistionPID.PID;
import frc.robot.Constants.Wrist.FeedForward;

public class PIDArmSubsystem extends ArmSubsystem {
    private PIDController pidController = new PIDController(PID.kP, PID.kI, PID.kD);
    private ArmFeedforward armFeedforward = new ArmFeedforward(ArbitraryFeedForward.kS, ArbitraryFeedForward.kG, ArbitraryFeedForward.kV);
    public PIDArmSubsystem() {
        super();
        SmartDashboard.putNumber("Arm P", PID.kP);
        pidController.disableContinuousInput();
        pidController.setSetpoint(getArmAngleInDegrees());
    }

    @Override
    public void periodic()
    {
        super.periodic();
        pidController.setP(SmartDashboard.getNumber("Arm P", PID.kP));
        pidController.setI(SmartDashboard.getNumber("Arm I", PID.kI));
        pidController.setD(SmartDashboard.getNumber("Arm D", PID.kD));
        SmartDashboard.putNumber("Arm error", pidController.getPositionError());
        SmartDashboard.putNumber("Arm setpoint", pidController.getSetpoint());

    }

    @Override
    public void setAngleInDegrees(double degrees) {
        pidController.setSetpoint(degrees);
    }

    public double calculateVoltage() {
        return MathUtil.clamp(pidController.calculate(getArmAngleInDegrees()), -3.75, 3.75);
    }
}
