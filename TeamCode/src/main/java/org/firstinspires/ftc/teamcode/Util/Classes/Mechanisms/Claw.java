package org.firstinspires.ftc.teamcode.Util.Classes.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Classes.Other.MicroServo;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ClawPosition;

public class Claw {
    private final Telemetry telemetry;

    public final MicroServo primaryClawServo;
    private final MicroServo secondaryClawServo;

    public boolean isOpen = false;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        primaryClawServo = new MicroServo(hardwareMap.get(Servo.class, "claw0"));
        secondaryClawServo = new MicroServo(hardwareMap.get(Servo.class, "claw1"));
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

    public double getPrimaryPosition() {
        return primaryClawServo.getPosition();
    }

    public double getSecondaryPosition() {
        return secondaryClawServo.getPosition();
    }
}
