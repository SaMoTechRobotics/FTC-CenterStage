package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ClawPosition;

public class Claw {
    private final Telemetry telemetry;

    private final Servo clawServo;

    private boolean clawOpen = false;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo.setPosition(ClawPosition.Closed);
    }

    public void setOpen(boolean open) {
        clawOpen = open;
        clawServo.setPosition(clawOpen ? ClawPosition.Open : ClawPosition.Closed);
    }

    public void toggle() {
        setOpen(!clawOpen);
    }
}
