package org.firstinspires.ftc.teamcode.util.auto;

public enum BoardPosition {
    INNER,
    CENTER,
    OUTER;

    public int[] getTagNumbers() {
        switch (this) {
            case INNER:
                return new int[]{1, 4};
            case CENTER:
                return new int[]{2, 5};
            case OUTER:
                return new int[]{3, 6};
            default:
                return new int[]{-1};
        }
    }
}
