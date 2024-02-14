package org.firstinspires.ftc.teamcode.util.auto;

public enum AutoSide {
    NEAR(-1),
    FAR(1);

    public final int value;

    AutoSide(int value) {
        this.value = value;
    }
}
