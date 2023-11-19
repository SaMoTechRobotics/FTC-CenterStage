package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;


@Config
@TeleOp(name = "ArmTest", group = "Tests")
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.claw.close();
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.claw.open();
            }

            if (Gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
                robot.arm.testWristDegrees(180, telemetry);
            } else if (Gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
                robot.arm.testWristDegrees(0, telemetry);
            }

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.B)) {
                robot.arm.setWristRotation(WristRotation.Down);
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                robot.arm.setWristRotation(WristRotation.Up);
            }

            Gamepad1.readButtons();
            Gamepad2.readButtons();
            telemetry.update();
        }
    }
}