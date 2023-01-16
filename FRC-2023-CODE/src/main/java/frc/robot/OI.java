package frc.robot;

//import java.lang.Thread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static final Joystick rightJoystickController = new Joystick(
            frc.robot.Constants.OI.kRightJoystickControllerPort);
    public static final Joystick leftJoystickController = new Joystick(
            frc.robot.Constants.OI.kLeftJoystickControllerPort);
    public static final XboxController xboxController = new XboxController(frc.robot.Constants.OI.kXboxControllerPort);

}
