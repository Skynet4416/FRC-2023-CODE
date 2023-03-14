package frc.robot.commands.Arm.PIDCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.PIDArmSubsystem;

public class ArmInstantCommand extends CommandBase{
    private final PIDArmSubsystem pidArmSubsystem;
    private final double angle;
    public ArmInstantCommand(PIDArmSubsystem armSubsystem, double angle)
    {
        this.pidArmSubsystem=  armSubsystem;
        this.angle = angle;
    }
    @Override
    public void initialize() {
       pidArmSubsystem.setAngleInDegrees(angle);
       end(false);
    }
}
