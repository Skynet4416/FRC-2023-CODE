package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Arm.Physical;
import frc.robot.Constants.Arm.PoistionPID.ArbitraryFeedForward;
import frc.robot.Constants.Arm.PoistionPID.PID;

public class PIDArmSubsystem extends ArmSubsystem {
    private PIDController pidController = new PIDController(PID.kP, PID.kI, PID.kD);

    public PIDArmSubsystem() {
        super();
        SmartDashboard.putNumber("Arm P", PID.kP);
        pidController.setSetpoint(getArmAngleInDegrees());
    }

    @Override
    public void periodic()
    {
        super.periodic();
        pidController.setP(SmartDashboard.getNumber("Arm P", PID.kP));
        SmartDashboard.putNumber("Arm error", pidController.getPositionError());
        SmartDashboard.putNumber("Arm goal", pidController.getSetpoint());

    }

    @Override
    public void setAngleInDegrees(double degrees) {
        pidController.setSetpoint(degrees);
    }

    public double calculateVoltage() {
        return pidController.calculate(getArmAngleInDegrees());
    }
}
