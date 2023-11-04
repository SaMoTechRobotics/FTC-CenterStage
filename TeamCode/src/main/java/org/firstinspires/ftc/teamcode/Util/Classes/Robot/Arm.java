package org.firstinspires.ftc.teamcode.Util.Classes.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ClawPosition;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;

public class Arm {
    private final Telemetry telemetry;

    private final DcMotor armMotor;
    private final Servo wristServo;
    private final Servo droneServo;

    private boolean globalWristRotation = false;

    private double lastLoopTime = 0;
    private double targetStartRotation = 0;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristServo = hardwareMap.get(Servo.class, "wrist");
        droneServo = hardwareMap.get(Servo.class, "drone");
        setWristRotation(WristRotation.Down);
        droneServo.setPosition(ClawPosition.DroneLock);

    }

    public void update() {
        if (globalWristRotation) updateGlobalWristRotation();

//        if (armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
//            updateSpeedRamping();
//        }
    }

    public void updateSpeedRamping() {
        telemetry.addData("target", armMotor.getTargetPosition());
        telemetry.addData("start", targetStartRotation);
        telemetry.addData("current", armMotor.getCurrentPosition());

        double period = Math.abs(armMotor.getTargetPosition() - targetStartRotation);
        double x = Math.abs(armMotor.getCurrentPosition());
        double h = Math.abs(targetStartRotation);


        double sin = Math.sin((Math.PI / (period / ArmRotation.PeriodDivider)) * (x - (period / ArmRotation.StartDivider) - h));
        telemetry.addData("sin", sin);

        double motorPower = ArmSpeed.Min * (
                sin
        ) + (1.0 - 2.0 * ArmSpeed.Min);

//        armMotor.setPower(motorPower);

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        dashboardTelemetry.addData("s", sin);
        dashboardTelemetry.addData("x", armMotor.getPower());
        dashboardTelemetry.update();

        armMotor.setPower(ArmSpeed.Max);

//            double loopTime = System.currentTimeMillis();
//            double timeDifference = loopTime - lastLoopTime;
//            lastLoopTime = loopTime;
//            double rampSpeed = ArmSpeed.RampSpeed * timeDifference;
//
//            boolean speedUp = armMotor.getCurrentPosition() > (armMotor.getTargetPosition() + targetStartRotation) / 2;
//
//            if(speedUp) {
//                if (armMotor.getPower() < ArmSpeed.Max) {
//                    if (armMotor.getPower() + rampSpeed > ArmSpeed.Max) {
//                        armMotor.setPower(ArmSpeed.Max);
//                    } else {
//                        armMotor.setPower(armMotor.getPower() + rampSpeed);
//                    }
//                }
//            } else {
//                if (armMotor.getPower() > ArmSpeed.Min) {
//                    if (armMotor.getPower() - rampSpeed > ArmSpeed.Min) {
//                        armMotor.setPower(ArmSpeed.Min);
//                    } else {
//                        armMotor.setPower(armMotor.getPower() - rampSpeed);
//                    }
//                }
//            }

//        TelemetryPacket graphSpeedRamping =



    }

    public void launchDrone() {
        droneServo.setPosition(ClawPosition.DroneRelease);
    }

    public double getRotation() {
        return armMotor.getCurrentPosition() * (90.0 / ArmRotation.TicksAt90Degrees);
    }

    public double getWristRotation() {
        return wristServo.getPosition();
    }

    public void setWristPickup(Boolean value) {
        if (value) {
            setRotation(ArmRotation.Down);
            setWristRotation(WristRotation.Down);
        } else {
            setRotation(ArmRotation.HoldDown);
            setWristRotation(WristRotation.HoldDown);
        }
    }

    public void holdRotation() {
        if (armMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ArmSpeed.HoldSpeed);
        }
    }

    public void setRotation(double degrees) {
        targetStartRotation = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(degreesToArmTicks(degrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double diff = Math.abs(degrees - armMotor.getCurrentPosition() * 90.0 / ArmRotation.TicksAt90Degrees);
        if (diff < 15) {
            armMotor.setPower(ArmSpeed.Min);
        } else if (diff < 45) {
            armMotor.setPower(ArmSpeed.Mid);
        } else {
            armMotor.setPower(ArmSpeed.Max);
        }
    }

    public void setRotation(double degrees, double speed) {
        armMotor.setTargetPosition(degreesToArmTicks(degrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(speed);
    }

    public void manualRotation(double power) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
    }

    public void setGlobalWristRotation(boolean value) {
        globalWristRotation = value;
    }

    public void updateGlobalWristRotation() {
        int ticks = armMotor.getCurrentPosition();
        double rotation = ticks * (90.0 / ArmRotation.TicksAt90Degrees);
        double wristRotation = rotation - WristRotation.ArmComplementAngle;
        double wristPosition = degreesToWristPosition(wristRotation);
        wristServo.setPosition(wristPosition);
    }

    public void setWristRotation(double position) {
        wristServo.setPosition(position);
        globalWristRotation = false;
    }

    public void setHangingLock(boolean lock) {
        if (lock) {
            armMotor.setTargetPosition(degreesToArmTicks(ArmRotation.HangingLock));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristServo.setPosition(WristRotation.Hang);
            armMotor.setPower(ArmSpeed.Max);
        } else {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(0);
        }
    }

    public void testWristDegrees(double degrees, Telemetry telemetry) {
        wristServo.setPosition(degreesToWristPosition(degrees));
        telemetry.addData("Wrist Position At " + degrees + " deg", degreesToWristPosition(degrees));
    }

    private int degreesToArmTicks(double degrees) {
        return (int) (degrees * (ArmRotation.TicksAt90Degrees / 90.0));
    }

    private double degreesToWristPosition(double degrees) {
        return (degrees * (Math.abs(WristRotation.PositionAt180Degrees - WristRotation.PositionAt0Degrees)) / 180.0 + WristRotation.PositionAt0Degrees);
    }
}
