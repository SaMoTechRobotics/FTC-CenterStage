package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Config
@TeleOp(name = "MotorTest", group = "Tests")
public class MotorTest extends LinearOpMode {
    public static Double SlowSpeed = 0.4;

    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "arm");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);

        telemetry.speak("Initialized Motor Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                motor.setPower(Gamepad1.getLeftY() * SlowSpeed);
            } else {
                motor.setPower(Gamepad1.getLeftY());
            }

            if (Gamepad1.getButton(GamepadKeys.Button.A)) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (Gamepad1.getButton(GamepadKeys.Button.B)) {
                if (motor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }


            telemetry.addData("Motor Ticks", motor.getCurrentPosition());
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addLine("---");
            telemetry.addData("Motor Zero Power Behavior", motor.getZeroPowerBehavior());
            telemetry.addData("Motor Mode", motor.getMode());
            telemetry.update();

            Gamepad1.readButtons();
        }
    }
}