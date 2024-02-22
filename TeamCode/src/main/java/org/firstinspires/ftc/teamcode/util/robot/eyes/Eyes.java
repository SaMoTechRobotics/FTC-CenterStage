package org.firstinspires.ftc.teamcode.util.robot.eyes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Eyes {
    private final Servo leftEye;
    private final Servo rightEye;

    private final Servo leftEyelid;
    private final Servo rightEyelid;

    public Eyes(HardwareMap hardwareMap) {
        leftEye = hardwareMap.get(Servo.class, "leftEye");
        rightEye = hardwareMap.get(Servo.class, "rightEye");
        leftEye.setDirection(Servo.Direction.REVERSE);

        leftEyelid = hardwareMap.get(Servo.class, "leftEyelid");
        rightEyelid = hardwareMap.get(Servo.class, "rightEyelid");
    }

    public void setEyes(double position) {
        leftEye.setPosition(position);
        rightEye.setPosition(position);
    }

    public void setEyelids(double position) {
        leftEyelid.setPosition(position);
        rightEyelid.setPosition(position);
    }

    public void blink() {
        setEyelids(EyelidPositions.Closed);
        setEyelids(EyelidPositions.Open);
    }

    public void open() {
        setEyelids(EyelidPositions.Open);
    }

    public void close() {
        setEyelids(EyelidPositions.Closed);
    }

    public void squint() {
        setEyelids(EyelidPositions.Squint);
    }

    public void lookForward() {
        setEyes(EyePositions.Forward);
    }

    public void lookLeft() {
        setEyes(EyePositions.Left);
    }

    public void lookRight() {
        setEyes(EyePositions.Right);
    }
}
