package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.GamepadButton;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.StatefulGamepad;


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

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        telemetry.speak("Initialized Motor Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
                motor.setPower(gamepad1.left_stick_y * SlowSpeed);
            } else {
                motor.setPower(gamepad1.left_stick_y);
            }

            if (gamepad1Buttons.getButton(GamepadButton.A)) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad1Buttons.getButton(GamepadButton.B)) {
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

            gamepad1Buttons.update();
        }
    }
}