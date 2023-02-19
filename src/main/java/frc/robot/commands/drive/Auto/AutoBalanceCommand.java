package frc.robot.commands.drive.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Drive.ChargeStationPID;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.util.AllianceFlipUtil;

public class AutoBalanceCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final PIDController pidController = new PIDController(ChargeStationPID.kP, ChargeStationPID.kI,
            ChargeStationPID.kD);

    /** Creates a new ChargeStationAutoBalance. */
    public AutoBalanceCommand(DriveSubsystem drive) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pidController.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double chargeStationAngle = drive.getChargeStationAngle();

        // P controller
        double driveSpeedAfterPID = pidController.calculate(chargeStationAngle);

        // charge station limits
        double chargeStationInnerX = AllianceFlipUtil.apply(FieldConstants.Community.chargingStationInnerX);
        double chargeStationOuterX = AllianceFlipUtil.apply(FieldConstants.Community.chargingStationOuterX);

        // speed limits depending on alliance
        if (DriverStation.getAlliance() == Alliance.Blue) {
            // check boundaries for left of charge station
            if (drive.getPosition().getX() <= chargeStationInnerX && driveSpeedAfterPID < 0) {
                driveSpeedAfterPID = 0;
            }

            // check boundaries for right of charge station
            if (drive.getPosition().getX() > chargeStationOuterX
                    && driveSpeedAfterPID > 0) {
                driveSpeedAfterPID = 0;
            }
        } else {
            // check boundaries for left of charge station
            if (drive.getPosition().getX() <= chargeStationInnerX && driveSpeedAfterPID > 0) {
                driveSpeedAfterPID = 0;
            }

            // check boundaries for right of charge station
            if (drive.getPosition().getX() > chargeStationOuterX
                    && driveSpeedAfterPID < 0) {
                driveSpeedAfterPID = 0;
            }
        }

        // command drive subsystem
        drive.setArcadeDrive(driveSpeedAfterPID, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
