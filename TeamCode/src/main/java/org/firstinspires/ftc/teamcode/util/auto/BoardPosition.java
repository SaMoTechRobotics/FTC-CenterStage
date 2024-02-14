package org.firstinspires.ftc.teamcode.util.auto;

public enum BoardPosition {
    LEFT,
    CENTER,
    RIGHT;

    public int[] getTagNumbers() {
        switch (this) {
            case LEFT:
                return new int[]{1, 4};
            case CENTER:
                return new int[]{2, 5};
            case RIGHT:
                return new int[]{3, 6};
            default:
                return new int[]{-1};
        }
    }
}
