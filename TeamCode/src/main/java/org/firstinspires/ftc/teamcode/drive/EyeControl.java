package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.util.robot.eyes.Eyes;


@Config
@TeleOp(name = "EyeControl")
@Disabled
public class EyeControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        Eyes eyes = new Eyes(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            gamepad1Buttons.update();
            gamepad2Buttons.update();


            if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                eyes.open();
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.B)) {
                eyes.close();
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
                eyes.squint();
            }

            if (gamepad1.left_stick_x != 0) {
                eyes.setEyes(0.5 + gamepad1.left_stick_x / 2);
            } else if (gamepad1Buttons.getButton(GamepadButton.LEFT_BUMPER)) {
                eyes.lookLeft();
            } else if (gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
                eyes.lookRight();
            } else {
                eyes.lookForward();
            }


            telemetry.update();

        }
    }
}