package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.GamepadButton;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;


@Config
@TeleOp(name = "ArmTest", group = "Tests")
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2Buttons.wasJustPressed(GamepadButton.RIGHT_BUMPER)) {
                robot.claw.close();
            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.LEFT_BUMPER)) {
                robot.claw.open();
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
                robot.arm.testWristDegrees(180, telemetry);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.arm.testWristDegrees(0, telemetry);
            }

            if (gamepad2Buttons.wasJustPressed(GamepadButton.B)) {
                robot.arm.setWristRotation(WristRotation.Down);
            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.Y)) {
                robot.arm.setWristRotation(WristRotation.Up);
            }

            gamepad1Buttons.update();
            gamepad2Buttons.update();
            telemetry.update();
        }
    }
}