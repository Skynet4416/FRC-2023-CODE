package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
        public static final Joystick rightJoystickController = new Joystick(
                        frc.robot.Constants.OIConstants.kRightJoystickControllerPort);
        public static final Joystick leftJoystickController = new Joystick(
                        frc.robot.Constants.OIConstants.kLeftJoystickControllerPort);
        public static final XboxController xboxController = new XboxController(
                        frc.robot.Constants.OIConstants.kXboxControllerPort);

        public static JoystickButton A = new JoystickButton(xboxController, XboxController.Button.kA.value);

}
