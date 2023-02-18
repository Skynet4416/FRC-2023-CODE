package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.DPadButton.DPadDirection;

public class OI {
        public static final Joystick rightJoystickController = new Joystick(
                        frc.robot.Constants.OI.kRightJoystickControllerPort);
        public static final Joystick leftJoystickController = new Joystick(
                        frc.robot.Constants.OI.kLeftJoystickControllerPort);
        public static final XboxController xboxController = new XboxController(
                        frc.robot.Constants.OI.kXboxControllerPort);

        public static final JoystickButton A = new JoystickButton(xboxController, XboxController.Button.kA.value);
        public static final JoystickButton B = new JoystickButton(xboxController, XboxController.Button.kB.value);
        public static final JoystickButton X = new JoystickButton(xboxController, XboxController.Button.kX.value);
        public static final JoystickButton Y = new JoystickButton(xboxController, XboxController.Button.kY.value);
        public static final DPadButton DPadUP = new DPadButton(xboxController, DPadDirection.UP);
        public static final DPadButton DPadDOWN = new DPadButton(xboxController, DPadDirection.DOWN);
        public static final DPadButton DpadRIGHT = new DPadButton(xboxController, DPadDirection.RIGHT);
        public static final DPadButton DPadLEFT = new DPadButton(xboxController, DPadDirection.LEFT);

}
