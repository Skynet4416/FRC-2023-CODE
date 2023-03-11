package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    // private final PneumaticsControlModule pcm = new PneumaticsControlModule();
    private final Solenoid on = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    private final Solenoid red = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid green = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private final Solenoid blue = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    private final Solenoid[] rgb = new Solenoid[] { red, green, blue };
    public LEDSubsystem(){
        // setColor(ColorEnum.BLUE);
    }
    public void setColor(ColorEnum colorEnum) {
        on.set(true);
        red.set(colorEnum.red);
        blue.set(colorEnum.blue);
        green.set(colorEnum.green);

    }
}
