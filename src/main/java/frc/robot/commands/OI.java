package frc.robot.commands;
import java.util.concurrent.TimeUnit;

//import java.lang.Thread;
import edu.wpi.first.wpilibj.Joystick;

public class OI{
    public static final Joystick Rjoystick = new Joystick(0);
    public static final Joystick Ljoystick = new Joystick(1);
    
    public boolean IsTriggered(){
        return Rjoystick.getTrigger() || Ljoystick.getTrigger();
    }

    public void printState(){
        while(true){
            System.out.println("x: " + this.Rjoystick.getX() + " y: " + this.Rjoystick.getY() + " z: " + this.Rjoystick.getZ());
        }
    }
    


}
