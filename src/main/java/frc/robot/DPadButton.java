package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DPadButton extends Trigger {
    public enum DPadDirection {
        UP(0),
        UPRIGHT(45),
        RIGHT(90),
        DOWNRIGHT(135),
        DOWN(180),
        DOWNLEFT(225),
        LEFT(270),
        UPLEFT(315);
        private int angle;
        DPadDirection(int angle) {
            this.angle = angle;
        }
        public int getAngle() {
            return angle;
        }
    }
    private XboxController controller;
    private DPadDirection direction;
    public DPadButton(XboxController controller, DPadDirection direction) {
        this.controller = controller;
        this.direction = direction;
    }

    public boolean get() {
        return controller.getPOV() == direction.getAngle();
    }
}