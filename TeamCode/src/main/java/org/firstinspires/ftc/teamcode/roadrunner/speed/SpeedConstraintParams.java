package org.firstinspires.ftc.teamcode.roadrunner.speed;

import com.acmerobotics.roadrunner.*;

import java.util.Arrays;

public class SpeedConstraintParams {
    private double maxWheelVel;

    private double maxAngVel;
    private double maxAngAccel;

    private double minProfileAccel;
    private double maxProfileAccel;

    /**
     * A class to hold the parameters for a SpeedConstraint
     *
     * @param maxWheelVel     The maximum velocity of the robot
     * @param maxAngVel       The maximum angular velocity of the robot
     * @param maxAngAccel     The maximum angular acceleration of the robot
     * @param minProfileAccel The minimum profile acceleration of the robot
     * @param maxProfileAccel The maximum profile acceleration of the robot
     */
    public SpeedConstraintParams(double maxWheelVel, double maxAngVel, double maxAngAccel, double minProfileAccel, double maxProfileAccel) {
        this.maxWheelVel = maxWheelVel;
        this.maxAngVel = maxAngVel;
        this.maxAngAccel = maxAngAccel;
        this.minProfileAccel = minProfileAccel;
        this.maxProfileAccel = maxProfileAccel;
    }

    public SpeedConstraint getSpeedConstraint(MecanumKinematics kinematics) {
        VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(maxWheelVel),
                new AngularVelConstraint(maxAngVel)
        ));
        AccelConstraint accelConstraint = new ProfileAccelConstraint(minProfileAccel, maxProfileAccel);
        return new SpeedConstraint(
                velConstraint,
                accelConstraint,
                new TurnConstraints(maxAngVel, -maxAngAccel, maxAngAccel)
        );
    }
}
