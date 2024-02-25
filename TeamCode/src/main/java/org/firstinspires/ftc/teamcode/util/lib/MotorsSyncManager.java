package org.firstinspires.ftc.teamcode.util.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controls the slide motors together to make sure they are always in sync
 */
public class MotorsSyncManager {
    private final DcMotor[] motors;

    public MotorsSyncManager(DcMotor... motors) {
        this.motors = motors;
    }

    public void setPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void setTargetPosition(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    public void setTargetPositionToCurrent() {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(motor.getCurrentPosition());
        }
    }

    public void setMode(DcMotor.RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public double getSyncError() {
        return Math.abs(motors[0].getCurrentPosition() - motors[1].getCurrentPosition());
    }

    public DcMotor.RunMode getMode() {
        return motors[0].getMode();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public int getCurrentPosition() {
        return motors[0].getCurrentPosition();
    }

    public int getTargetPosition() {
        return motors[0].getTargetPosition();
    }

    @Override
    public String toString() {
        StringBuilder log = new StringBuilder("MotorsSyncManager: \n");
        for (DcMotor motor : motors) {
            log.append("Mode: ").append(motor.getMode()).append("\n");
            log.append("Power: ").append(motor.getPower()).append("\n");
            log.append("Position: ").append(motor.getCurrentPosition()).append("\n");
            log.append("Target Position: ").append(motor.getTargetPosition()).append("\n");
            log.append("Zero Power Behavior: ").append(motor.getZeroPowerBehavior()).append("\n\n");
        }
        return log.toString();
    }
}
