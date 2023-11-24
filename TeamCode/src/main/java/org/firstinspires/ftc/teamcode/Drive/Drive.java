package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.FieldOverlayUtils;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.GamepadButton;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ChassisSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Timer;


@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    public static boolean DebuggingTelemetry = false;
    public static boolean FieldOverlay = false;

    public static boolean AutoClose = false;

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.claw.close();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        VisionPortal visionPortal = builder.build();

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        Timer timer = new Timer();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_UP)) {
                visionPortal.resumeStreaming();
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_DOWN)) {
                visionPortal.stopStreaming();
            }

            if (gamepad1Buttons.stateJustChanged(GamepadButton.LEFT_BUMPER) || gamepad1Buttons.stateJustChanged(GamepadButton.RIGHT_BUMPER)) {
                robot.chassis.updateSpeed(
                        gamepad1Buttons.getButton(GamepadButton.LEFT_BUMPER),
                        gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)
                );
            }

            double drivePower = Math.abs(gamepad1.left_stick_y) > ChassisSpeed.JoystickYMargin ? gamepad1.left_stick_y : 0;
            double strafePower = Math.abs(gamepad1.left_stick_x) > ChassisSpeed.JoystickXMargin ? gamepad1.left_stick_x : 0;
            double turnPower = Math.abs(gamepad1.right_stick_x) > ChassisSpeed.JoystickXMargin ? gamepad1.right_stick_x : 0;

            boolean anyActiveJoystick = drivePower != 0 || strafePower != 0 || turnPower != 0;

//            if (gamepad1.left_trigger > 0.1 && !anyActiveJoystick && robot.claw.isOpen) {
//                if (robot.chassis.pixelDetectedInClaw()) {
//                    robot.claw.close();
//                } else if (robot.chassis.pixelDetected()) {
//                    robot.chassis.alignWithPixel();
//                }
//            }

            robot.chassis.setManualPower(drivePower, strafePower, turnPower);

            if (Math.abs(gamepad2.right_stick_x) > 0.01) {
                robot.arm.manualRotation(gamepad2.right_stick_x * ArmSpeed.SlowManual);
                if (!robot.pickUp) {
                    robot.arm.setGlobalWristRotation(true);
                }
            } else if (Math.abs(gamepad2.left_stick_x) > 0.01) {
                robot.arm.manualRotation(gamepad2.left_stick_x);
                if (!robot.pickUp) {
                    robot.arm.setGlobalWristRotation(true);
                }
            } else if (gamepad2.left_trigger < 0.1) {
                if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_UP)) {
                    robot.arm.setRotation(ArmRotation.HighDeliver);
                    robot.arm.setGlobalWristRotation(true);
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_LEFT)) {
                    robot.arm.setRotation(ArmRotation.MidDeliver);
                    robot.arm.setGlobalWristRotation(true);
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_DOWN)) {
                    robot.arm.setRotation(ArmRotation.LowDeliver);
                    robot.arm.setGlobalWristRotation(true);
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_RIGHT)) {
                    robot.arm.setRotation(ArmRotation.Down);
                    robot.arm.setWristPickup(true);
                } else if (gamepad1Buttons.wasJustReleased(GamepadButton.B)) {
                    robot.arm.setRotation(ArmRotation.Hang);
                    robot.arm.setWristRotation(WristRotation.Hang);
                } else {
                    robot.arm.holdRotation();
                }
            }

            if (gamepad2.left_trigger >= 0.1) {
                if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_UP)) {
                    robot.arm.setRotation(ArmRotation.Stack5);
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_LEFT)) {
                    robot.arm.setRotation(ArmRotation.Stack4);
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_DOWN)) {
                    robot.arm.setRotation(ArmRotation.Stack3);
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_RIGHT)) {
                    robot.arm.setRotation(ArmRotation.Down);
                }
            }

            if (gamepad2Buttons.wasJustReleased(GamepadButton.A)) {
                robot.resetForIntake();
            }

            if (gamepad2Buttons.wasJustPressed(GamepadButton.RIGHT_BUMPER)) {
                if (robot.pickUp && !robot.claw.isOpen && robot.arm.getRotation() < ArmRotation.HoldDown) {
                    robot.arm.setRotation(ArmRotation.HoldDown);
                    robot.arm.setWristRotation(WristRotation.HoldDown);
                }
                robot.claw.close();
            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.LEFT_BUMPER)) {
                if (robot.pickUp) {
                    if (Math.floor(robot.arm.getRotation()) <= ArmRotation.HoldDown + 1.0) {
                        robot.arm.setRotation(ArmRotation.Down);
                    }
                    robot.claw.open();
                } else {
                    robot.claw.openNext();
                }
            } else if (AutoClose) {
                if (robot.chassis.pixelDetectedInClaw() && robot.claw.isOpen) {
                    robot.claw.close();
                }
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
                robot.arm.setHangingLock(true);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.arm.setHangingLock(false);
            }

            if (gamepad2Buttons.wasJustPressed(GamepadButton.B)) {
                robot.arm.setWristRotation(WristRotation.Down);
            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.Y)) {
                robot.arm.setWristRotation(WristRotation.Up);
            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.X)) {
                robot.arm.setWristRotation(WristRotation.Forward);
            }

            if (gamepad1.left_trigger > 0.1 && gamepad1Buttons.wasJustPressed(GamepadButton.LEFT_BUMPER)) {
                robot.prepareDroneLaunch();
            }
            if (gamepad1.right_trigger > 0.1 && gamepad1Buttons.wasJustPressed(GamepadButton.RIGHT_BUMPER) && robot.droneReady) {
                robot.arm.launchDrone();
            }

            robot.update();
            telemetry.addData("Robot arm rotation", robot.arm.getRotation());

            gamepad1Buttons.update();
            gamepad2Buttons.update();
            telemetry.update();

            if (FieldOverlay) {
                drawFieldOverlay();
            }
        }
    }

    private void drawFieldOverlay() {
        TelemetryPacket packet = new TelemetryPacket();
        FieldOverlayUtils.drawChassis(packet.fieldOverlay(), robot.chassis.getPose());
        FieldOverlayUtils.drawPixel(packet.fieldOverlay(), new Vector2d(32, 0));

//        packet.fieldOverlay().fillRect(0, 0, 20, 5);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}