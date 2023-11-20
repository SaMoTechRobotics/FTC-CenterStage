package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Timer;


@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    public static boolean DebuggingTelemetry = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.claw.close();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        VisionPortal visionPortal = builder.build();

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        Timer timer = new Timer();

        waitForStart();

        while (opModeIsActive()) {
            if (Gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                visionPortal.resumeStreaming();
            } else if (Gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                visionPortal.stopStreaming();
            }

            robot.chassis.updateSpeed(
                    Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
                    Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
            );

            robot.chassis.updateWithControls(Gamepad1, Gamepad2, !robot.pickUp);

            if (Math.abs(Gamepad2.getRightX()) > 0.01) {
                robot.arm.manualRotation(Gamepad2.getRightX() * ArmSpeed.SlowManual);
                if (!robot.pickUp) {
                    robot.arm.setGlobalWristRotation(true);
                }
            } else if (Math.abs(Gamepad2.getLeftX()) > 0.01) {
                robot.arm.manualRotation(Gamepad2.getLeftX());
                if (!robot.pickUp) {
                    robot.arm.setGlobalWristRotation(true);
                }
            } else if (Gamepad2.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                robot.arm.setRotation(ArmRotation.HighDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                robot.arm.setRotation(ArmRotation.MidDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                robot.arm.setRotation(ArmRotation.LowDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
                robot.arm.setRotation(ArmRotation.Down);
                robot.arm.setWristPickup(true);
            } else if (Gamepad1.wasJustReleased(GamepadKeys.Button.B)) {
                robot.arm.setRotation(ArmRotation.Hang);
                robot.arm.setWristRotation(WristRotation.Hang);
            } else {
                robot.arm.holdRotation();
            }

            if (Gamepad2.wasJustReleased(GamepadKeys.Button.A)) {
                robot.resetForIntake();
            }

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (robot.pickUp && !robot.claw.isOpen && robot.arm.getRotation() < ArmRotation.HoldDown) {
                    robot.arm.setRotation(ArmRotation.HoldDown);
                    robot.arm.setWristRotation(WristRotation.HoldDown);
                }
                robot.claw.close();
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                if (robot.pickUp) {
                    if (Math.floor(robot.arm.getRotation()) <= ArmRotation.HoldDown + 1.0) {
                        robot.arm.setRotation(ArmRotation.Down);
                    }
                    robot.claw.open();
                } else {
                    robot.claw.openNext();
                }
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
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.X)) {
                robot.arm.setWristRotation(WristRotation.Forward);
            }

            if (Gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 && Gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.prepareDroneLaunch();
            }
            if (Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 && Gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && robot.droneReady) {
                robot.arm.launchDrone();
            }

            robot.update();
            telemetry.addData("Robot arm rotation", robot.arm.getRotation());

            telemetry.addData("Secondary servo position", robot.claw.getSecondaryPosition());

            Gamepad1.readButtons();
            Gamepad2.readButtons();
            telemetry.update();
        }
    }
}