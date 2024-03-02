package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@Autonomous(name = "IMU Test", group = "Tests")
public class IMUTest extends LinearOpMode {
    public static double startAngle = 90;

    public static double offsetAngle = 0;

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double fieldAngle = angle + startAngle + offsetAngle;
            telemetry.addData("IMU (RAW DEGREES)", angle);
            telemetry.addData("IMU (FIELD DEGREES)", fieldAngle);
            telemetry.addLine("---");
            telemetry.addData("Loop", timer.milliseconds() + "ms");
            telemetry.update();

            timer.reset();
        }
    }
}