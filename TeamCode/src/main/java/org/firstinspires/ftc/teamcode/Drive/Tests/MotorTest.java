package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmSpeed;


@Config
@TeleOp(name = "MotorTest", group = "Tests")
public class MotorTest extends LinearOpMode {
    public static Double ClawOpen = 0.4;
    public static Double ClawClosed = 0.05;

    public static Double WristDown = 0.75;
    public static Double WristUp = 0.2;

    public static int ArmDeliverPosition = -425;
    public static int ArmHangPosition = -280;

    public static Double armSpeed = 0.15;

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap, telemetry);

        DcMotor motor = hardwareMap.get(DcMotor.class, "arm");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        boolean clawOpen = false;

        waitForStart();

        while (opModeIsActive()) {
            chassis.updateSpeed(
                    Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
                    Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
            );

            chassis.updateWithControls(Gamepad1);

            if (Gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                motor.setPower(Gamepad2.getLeftY() * ArmSpeed.Max);
            } else {
                motor.setPower(Gamepad2.getLeftY() * armSpeed);
            }

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.X)) {
                clawOpen = !clawOpen;
                claw.setPosition(clawOpen ? ClawOpen : ClawClosed);
            }

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                wrist.setPosition(wrist.getPosition() == WristDown ? WristUp : WristDown);
            }

//
//            if(Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                motor.setTargetPosition(ArmDeliverPosition);
//                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor.setPower(armSpeed);
//            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//                motor.setTargetPosition(ArmHangPosition);
//                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor.setPower(armSpeed);
//            } else if(Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                motor.setTargetPosition(0);
//                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor.setPower(armSpeed);
//            }


            telemetry.addData("Motor Ticks", motor.getCurrentPosition());
            telemetry.update();

            Gamepad2.readButtons();
        }
    }
}