package org.firstinspires.ftc.teamcode.util.lib;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class MicroServo extends ServoImpl {

    //Formula = (Pulse Width - 500) / 2000
    public static double min = 0.125;
    public static double max = 0.875;

    public MicroServo(Servo servo) {
        super(servo.getController(), servo.getPortNumber());
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(Math.max(min, Math.min(max, position)));
    }
}
