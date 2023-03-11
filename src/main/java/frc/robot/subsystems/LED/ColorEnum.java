package frc.robot.subsystems.LED;



public enum ColorEnum {
    //
    BLUE(false, false, true),
    GREEN(false, true, false),
    RED(true, false, false),
    //
    WHITE(true, true, true),
    BLACK(false, false, false),
    //
    PERPULE(true, false, true),
    YELLOW(true, true, false),
    CYAN(false, true, true);

    public final boolean blue, green, red;

    ColorEnum(boolean red, boolean green, boolean blue) {
        this.blue = blue;
        this.red = red;
        this.green = green;
    }

    public boolean[] getColorsArray() {
        return new boolean[] { red, green, blue };
    }
}
