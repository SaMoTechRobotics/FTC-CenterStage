package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.Robot;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ClawPosition;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;

import java.util.Timer;
import java.util.TimerTask;


@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        Timer timer = new Timer();

        waitForStart();

        while (opModeIsActive()) {

            robot.chassis.updateSpeed(
                    Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
                    Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
            );

            robot.chassis.updateWithControls(Gamepad1);

            if(Math.abs(Gamepad2.getRightX()) > 0.01) {
                robot.arm.manualRotation(Gamepad2.getRightX() * ArmSpeed.SlowManual);
                if (!robot.pickUp) {
                    robot.arm.setGlobalWristRotation(true);
                }
            } else if (Math.abs(Gamepad2.getLeftX()) > 0.01) {
                robot.arm.manualRotation(Gamepad2.getLeftX());
                if (!robot.pickUp) {
                    robot.arm.setGlobalWristRotation(true);
                }
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                robot.arm.setRotation(ArmRotation.HighDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robot.arm.setRotation(ArmRotation.MidDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robot.arm.setRotation(ArmRotation.LowDeliver);
                robot.arm.setGlobalWristRotation(true);
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                robot.arm.setRotation(ArmRotation.Down);
                robot.arm.setWristPickup(true);
            } else if (Gamepad1.wasJustPressed(GamepadKeys.Button.B)) {
                robot.arm.setRotation(ArmRotation.Hang);
                robot.arm.setWristRotation(WristRotation.Down);
            } else {
                robot.arm.holdRotation();
            }

            if (Gamepad2.getButton(GamepadKeys.Button.A)) {
                robot.resetForIntake();
            }

            if(Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.claw.setOpen(false);
                if (robot.pickUp) {
                    timer.purge();
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            robot.arm.setWristPickup(false);
                        }
                    }, ClawPosition.WaitTime);
                }
            } else if(Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.claw.setOpen(true);
                if (robot.pickUp && robot.arm.getWristRotation() != WristRotation.Forward) {
                    timer.purge();
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            robot.arm.setWristPickup(true);
                        }
                    }, ClawPosition.WaitTime);
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

            if(Gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 && Gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.prepareDroneLaunch();
            }
            if(Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 && Gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.arm.launchDrone();
            }

            robot.update();
            telemetry.addData("Robot arm rotation", robot.arm.getRotation());

            Gamepad1.readButtons();
            Gamepad2.readButtons();
            telemetry.update();
        }
    }
}