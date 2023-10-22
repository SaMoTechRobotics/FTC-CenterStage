package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.WristRotation;


@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            robot.chassis.updateSpeed(
                    Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
                    Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
            );

            robot.chassis.updateWithControls(Gamepad1);

            if (Gamepad2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                robot.arm.setRotation(ArmRotation.Down);
            } else if (Gamepad2.getButton(GamepadKeys.Button.DPAD_UP)) {
                robot.arm.setRotation(ArmRotation.Deliver);
            } else if (Gamepad2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                robot.arm.setRotation(ArmRotation.Hang);
            }

            if(Gamepad2.wasJustPressed(GamepadKeys.Button.X)) {
                robot.claw.toggle();
            }

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.A)) {
                robot.arm.setWristRotation(WristRotation.Down);
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                robot.arm.setWristRotation(WristRotation.Up);
            }

            Gamepad1.readButtons();
            Gamepad2.readButtons();
        }
    }
}