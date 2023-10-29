package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;


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

            if (Math.abs(Gamepad2.getLeftX()) > 0.01) {
                robot.arm.manualRotation(Gamepad2.getLeftX());
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.getButton(GamepadKeys.Button.DPAD_UP)) {
                robot.arm.setRotation(ArmRotation.HighDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                robot.arm.setRotation(ArmRotation.MidDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                robot.arm.setRotation(ArmRotation.LowDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad1.getButton(GamepadKeys.Button.B)) {
                robot.arm.setWristRotation(ArmRotation.Hang);
                robot.arm.setWristRotation(WristRotation.Down);
            } else {
                robot.arm.holdRotation();
            }

            if (Gamepad2.getButton(GamepadKeys.Button.A)) {
                robot.resetForIntake();
            }

            robot.arm.update();

            if(Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.claw.setOpen(false);
            } else if(Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.claw.setOpen(true);
            }


            if (Gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
                robot.arm.setHangingLock(true);
            } else if (Gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
                robot.arm.setHangingLock(false);
            }

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.B)) {
                robot.arm.setWristRotation(WristRotation.Down);
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                robot.arm.setWristRotation(WristRotation.Up);
            }

            if(Gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 && Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.prepareDroneLaunch();
            }
            if(Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 && Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.resetForIntake();
            }

            Gamepad1.readButtons();
            Gamepad2.readButtons();
            telemetry.update();
        }
    }
}