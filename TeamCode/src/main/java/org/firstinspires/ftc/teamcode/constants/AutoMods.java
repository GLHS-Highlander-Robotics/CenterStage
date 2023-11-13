package org.firstinspires.ftc.teamcode.constants;

public class AutoMods {
    public enum Locs {
        BLUE, RED, STAGE, AUDIENCE
    }
    public static Locs COLOR = Locs.BLUE;

    public static void setCOLOR(Locs COLOR) {
        AutoMods.COLOR = COLOR;
    }
}
