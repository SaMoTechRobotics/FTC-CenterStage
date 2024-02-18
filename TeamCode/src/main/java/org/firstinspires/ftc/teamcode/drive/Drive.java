package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.lib.FieldOverlayUtils;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.util.robot.Robot;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmSpeed;
import org.firstinspires.ftc.teamcode.util.robot.arm.WristRotation;
import org.firstinspires.ftc.teamcode.util.robot.chassis.ChassisSpeed;


@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    public static boolean FieldOverlay = false;
    public static boolean ResetPose = true;

    private Robot robot;

    @Override
    public void runOpMode() {
        if (ResetPose) RobotStorage.reset();
        robot = new Robot(hardwareMap);
        robot.arm.lockDrone();
        robot.claw.open();

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad Updates

            gamepad1Buttons.update();
            gamepad2Buttons.update();

            // Driving Speed Controls (Gamepad 1)

            if (gamepad1Buttons.stateJustChanged(GamepadButton.LEFT_BUMPER) || gamepad1Buttons.stateJustChanged(GamepadButton.RIGHT_BUMPER)) {
                robot.chassis.updateSpeed(
                        gamepad1Buttons.getButton(GamepadButton.LEFT_BUMPER),
                        gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)
                );
            }

            // Drive Controls (Gamepad 1)

            robot.chassis.setManualPower(
                    ChassisSpeed.applyDeadZone(gamepad1.left_stick_y, ChassisSpeed.DriveDeadZone),
                    ChassisSpeed.applyDeadZone(gamepad1.left_stick_x, ChassisSpeed.StrafeDeadZone),
                    ChassisSpeed.applyDeadZone(gamepad1.right_stick_x, ChassisSpeed.TurnDeadZone)
            );

            // Stack Presets (Gamepad 1)

            if (robot.pickUp) {
                if (gamepad1Buttons.wasJustReleased(GamepadButton.DPAD_UP)) {
                    robot.arm.setRotation(ArmRotation.Stack5);
                    robot.arm.setWristRotation(WristRotation.StackDown);
                    robot.claw.open();
                } else if (gamepad1Buttons.wasJustReleased(GamepadButton.DPAD_LEFT)) {
                    robot.arm.setRotation(ArmRotation.Stack4);
                    robot.arm.setWristRotation(WristRotation.StackDown);
                    robot.claw.open();
                } else if (gamepad1Buttons.wasJustReleased(GamepadButton.DPAD_DOWN)) {
                    robot.arm.setRotation(ArmRotation.Stack3);
                    robot.arm.setWristRotation(WristRotation.Down);
                    robot.claw.open();
                } else if (gamepad1Buttons.wasJustReleased(GamepadButton.DPAD_RIGHT)) {
                    robot.arm.setRotation(ArmRotation.Down);
                    robot.arm.setWristRotation(WristRotation.Down);
                    robot.claw.open();
                }
            }

            // Arm Controls (Gamepad 2)

            if (Math.floor(robot.arm.getRotation()) > ArmRotation.MinDeliver && Math.abs(gamepad2.left_stick_y) > 0.01) {
                if (gamepad2.left_trigger > 0.1) {
                    robot.arm.manualRotation(-gamepad2.left_stick_y * ArmSpeed.DeliverSpeed);
                } else {
                    robot.arm.manualRotation(-gamepad2.left_stick_y * ArmSpeed.SlowDeliverSpeed);
                }
                robot.arm.setGlobalWristRotation(robot.wristLevelingEnabled);
            } else if (robot.pickUp && Math.abs(gamepad2.right_stick_y) > 0.01) {
                if (gamepad2.left_trigger > 0.1) {
                    robot.arm.manualRotation(gamepad2.right_stick_y * ArmSpeed.PickupSpeed);
                } else {
                    robot.arm.manualRotation(gamepad2.right_stick_y * ArmSpeed.SlowPickupSpeed);
                }
                robot.arm.setGlobalWristRotation(false);
            } else {
//                else if (gamepad2.right_trigger < 0.1) {
                if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_UP)) {
                    robot.arm.setRotation(ArmRotation.HighDeliver);
                    robot.arm.setGlobalWristRotation(true);
                    robot.wristLevelingEnabled = true;
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_LEFT)) {
                    robot.arm.setRotation(ArmRotation.MidDeliver);
                    robot.arm.setGlobalWristRotation(true);
                    robot.wristLevelingEnabled = true;
                } else if (gamepad2Buttons.wasJustReleased(GamepadButton.DPAD_DOWN)) {
                    robot.arm.setRotation(ArmRotation.LowDeliver);
                    robot.arm.setGlobalWristRotation(true);
                    robot.wristLevelingEnabled = true;
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

            if (gamepad2Buttons.wasJustReleased(GamepadButton.A)) {
                robot.resetForIntake();
            }


            // Claw Controls (Gamepad 2)

            if (gamepad2Buttons.wasJustPressed(GamepadButton.RIGHT_BUMPER)) {
                if (robot.pickUp && !robot.claw.isOpen && robot.arm.getRotation() < ArmRotation.HoldDown) {
                    robot.arm.setRotation(ArmRotation.HoldDown);
                    robot.arm.setWristRotation(WristRotation.HoldDown);
                }
                robot.claw.close();
            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.LEFT_BUMPER)) {
                if (robot.pickUp) {
                    if (robot.claw.isOpen) {
                        robot.arm.setRotation(ArmRotation.Down);
                    }
                    robot.claw.open();
                } else {
                    robot.claw.openNext();
                }
            }

            // Hang Controls (Gamepad 1)

            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
                robot.arm.setHangingLock(true);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.arm.setHangingLock(false);
            }

            // Wrist Controls (Gamepad 2)

            if (gamepad2Buttons.wasJustPressed(GamepadButton.B)) {
                robot.arm.setWristRotation(WristRotation.Down);
            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.Y)) {
                robot.wristLevelingEnabled = false;
                robot.arm.setGlobalWristRotation(false);
                robot.arm.setWristRotation(WristRotation.PickupBack);
                robot.arm.setRotation(ArmRotation.BackDown, ArmSpeed.DeliverSpeed);
            }
//            else if (gamepad2Buttons.wasJustPressed(GamepadButton.X)) {
//                robot.wristLevelingEnabled = true;
//                robot.arm.setRotation(ArmRotation.LowDeliver);
//                robot.arm.setGlobalWristRotation(true);
//            }

            // Drone Launcher Controls (Gamepad 1)

            if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1 && gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
                robot.arm.launchDrone();
            }

            // Updates

            robot.update();
            if (robot.pickUp) {
                robot.wristLevelingEnabled = true;
                robot.claw.setFingerEnabled(false);
            } else {
                robot.claw.setFingerEnabled(!robot.claw.isOpen);
            }

            // Telemetry

            telemetry.addData("Robot arm rotation", robot.arm.getRotation());
            telemetry.addData("Robot arm ticks", robot.arm.getArmTicks());
            telemetry.update();

            if (FieldOverlay) {
                drawFieldOverlay();
            }
        }
    }

    private void drawFieldOverlay() {
        TelemetryPacket packet = new TelemetryPacket();
        FieldOverlayUtils.drawChassis(packet.fieldOverlay(), robot.chassis.getPose());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}