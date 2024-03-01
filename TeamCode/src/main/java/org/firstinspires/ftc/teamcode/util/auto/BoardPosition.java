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

    public int getIndex() {
        switch (this) {
            case INNER:
                return 0;
            case CENTER:
                return 1;
            case OUTER:
                return 2;
            default:
                return -1;
        }
    }

    public String getPlace(AutoColor color) {
        if (color == AutoColor.RED) {
            switch (this) {
                case INNER:
                    return "LEFT";
                case CENTER:
                    return "CENTER";
                case OUTER:
                    return "RIGHT";
            }
        } else {
            switch (this) {
                case INNER:
                    return "RIGHT";
                case CENTER:
                    return "CENTER";
                case OUTER:
                    return "LEFT";
            }
        }
        return "UNKNOWN";
    }
}
