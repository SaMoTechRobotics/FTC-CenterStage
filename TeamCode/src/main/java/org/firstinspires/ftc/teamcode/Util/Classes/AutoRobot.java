package org.firstinspires.ftc.teamcode.Util.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Classes.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Util.Classes.Mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.Util.Classes.Mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Util.Classes.Vision.Vision;

public class AutoRobot {
    private final Telemetry telemetry;

    public Vision vision;
    public Chassis chassis;
    public Arm arm;
    public Claw claw;

    public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        vision = new Vision(hardwareMap);
        chassis = new Chassis(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);

        this.telemetry = telemetry;
    }
}
