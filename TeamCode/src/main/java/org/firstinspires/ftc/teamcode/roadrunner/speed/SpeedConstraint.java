package org.firstinspires.ftc.teamcode.roadrunner.speed;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

public class SpeedConstraint {
    public final VelConstraint velConstraint;
    public final AccelConstraint accelConstraint;
    public final TurnConstraints turnConstraints;

    public SpeedConstraint(VelConstraint velConstraint, AccelConstraint accelConstraint, TurnConstraints turnConstraints) {
        this.velConstraint = velConstraint;
        this.accelConstraint = accelConstraint;
        this.turnConstraints = turnConstraints;
    }
}
