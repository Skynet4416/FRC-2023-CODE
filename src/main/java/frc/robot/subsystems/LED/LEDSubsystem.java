package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    // private final PneumaticsControlModule pcm = new PneumaticsControlModule();
    private final Solenoid red = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid green = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private final Solenoid blue = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    private final Solenoid[] rgb = new Solenoid[] { red, green, blue };
    public LEDSubsystem(){
        setColor(ColorEnum.BLUE);
    }
    public void setColor(ColorEnum colorEnum) {
        for (int i = 0; i < rgb.length; i++) {
            rgb[i].set(colorEnum.getColorsArray()[i]);
        }
    }
}
