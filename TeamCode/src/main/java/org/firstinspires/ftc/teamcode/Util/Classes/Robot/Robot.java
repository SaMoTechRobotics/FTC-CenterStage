package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Chassis chassis;
    public Arm arm;
    public Claw claw;

    public Robot(HardwareMap hardwareMap) {
        chassis = new Chassis(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
    }
}
