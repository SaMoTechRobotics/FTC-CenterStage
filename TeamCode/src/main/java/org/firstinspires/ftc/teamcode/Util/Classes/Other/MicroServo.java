package org.firstinspires.ftc.teamcode.Util.Classes.Other;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class MicroServo extends ServoImpl {
    public static Double min = 0.125;
    public static Double max = 0.875;

    public MicroServo(Servo servo) {
        super(servo.getController(), servo.getPortNumber());
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(Math.max(min, Math.min(max, position)));
    }
}
