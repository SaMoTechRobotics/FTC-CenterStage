package org.firstinspires.ftc.teamcode.util.robot.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public final Servo primaryClawServo;
    private final Servo secondaryClawServo;

    private final Servo fingerServo;

    public boolean isOpen = false;
    private boolean fingerEnabled = false;

    public Claw(HardwareMap hardwareMap) {
        primaryClawServo = hardwareMap.get(Servo.class, "claw0");
        secondaryClawServo = hardwareMap.get(Servo.class, "claw1");

        fingerServo = hardwareMap.get(Servo.class, "finger");
        fingerServo.setPosition(ClawPosition.FingerIn);
    }

    public void close() {
        isOpen = false;
        primaryClawServo.setPosition(ClawPosition.PrimaryClosed);
        secondaryClawServo.setPosition(ClawPosition.SecondaryClosed);
    }

    public void open() {
        isOpen = true;
        primaryClawServo.setPosition(ClawPosition.PrimaryOpen);
        secondaryClawServo.setPosition(ClawPosition.SecondaryOpen);
    }

    public void openNext() {
        if (Math.abs(primaryClawServo.getPosition() - ClawPosition.PrimaryOpen) > Math.abs(primaryClawServo.getPosition() - ClawPosition.PrimaryClosed)) {
            primaryClawServo.setPosition(ClawPosition.PrimaryOpen);
        } else {
            primaryClawServo.setPosition(ClawPosition.PrimaryOpen);
            secondaryClawServo.setPosition(ClawPosition.SecondaryOpen);
            isOpen = true;
        }
    }

    public void setFingerEnabled(boolean enabled) {
//        if (fingerEnabled == enabled) return;
        fingerEnabled = enabled;
        fingerServo.setPosition(enabled ? ClawPosition.FingerOut : ClawPosition.FingerIn);
    }

    public double getPrimaryPosition() {
        return primaryClawServo.getPosition();
    }

    public double getSecondaryPosition() {
        return secondaryClawServo.getPosition();
    }
}
