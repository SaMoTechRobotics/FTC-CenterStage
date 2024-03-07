package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.speed.SpeedConstraint;
import org.firstinspires.ftc.teamcode.roadrunner.speed.SpeedConstraintParams;
import org.firstinspires.ftc.teamcode.roadrunner.speed.TrajectorySpeed;

import java.lang.Math;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public static class Params {
        // drive model parameters
        public double inPerTick = 0.0005277604;
        public double lateralInPerTick = 0.0005758376975630854;
        public double trackWidthTicks = 18791.697458330193;

        //Coefficient for proportionate heading correction. This is just a guess.
        //If you see a lot of oscillation or bot spins out of control, try smaller.
        public double headingCorrectionCoefficient = 1;

        // feedforward parameters (in tick units)
        public double kS = 1.3572340058093988;
        public double kV = 0.00007475484378017362;
        public double kA = 0.00001;

        // Speed constraint parameters

        public SpeedConstraintParams defaultSpeeds = new SpeedConstraintParams(
                70, 1 * Math.PI, 0.5 * Math.PI, -40, 60
        );

        public SpeedConstraintParams slowSpeeds = new SpeedConstraintParams(
                20, Math.PI / 4, Math.PI / 2, -20, 20
        );

        public SpeedConstraintParams alignSpeeds = new SpeedConstraintParams(
                20, Math.PI / 4, Math.PI / 4, -20, 20
        );

        public SpeedConstraintParams fastSpeeds = new SpeedConstraintParams(
                80, 1 * Math.PI, 0.5 * Math.PI, -50, 70
        );


        // Old roadrunner parameters

//        // path profile parameters (in inches)
//        public double maxWheelVel = 80; // 20
//        public double minProfileAccel = -30;
//        public double maxProfileAccel = 60; //50
//
//        // turn profile parameters (in radians)
//        public double maxAngVel = Math.PI; // shared with path
//        public double maxAngAccel = Math.PI;


        // path controller gains
        public double axialGain = 10.0;
        public double lateralGain = 20.0;
        public double headingGain = 20.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    // Old roadrunner constraints

//    public TurnConstraints defaultTurnConstraints = new TurnConstraints(
//            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
//    public VelConstraint defaultVelConstraint =
//            new MinVelConstraint(Arrays.asList(
//                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
//                    new AngularVelConstraint(PARAMS.maxAngVel)
//            ));
//    public AccelConstraint defaultAccelConstraint =
//            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    // Custom speed constraints

    public final SpeedConstraint defaultSpeedConstraint = PARAMS.defaultSpeeds.getSpeedConstraint(kinematics);

    public final SpeedConstraint slowSpeedConstraint = PARAMS.slowSpeeds.getSpeedConstraint(kinematics);

    public final SpeedConstraint fastSpeedConstraint = PARAMS.fastSpeeds.getSpeedConstraint(kinematics);

    public final SpeedConstraint alignSpeedConstraint = PARAMS.alignSpeeds.getSpeedConstraint(kinematics);

    private final TrajectorySpeed defaultTrajectorySpeed = TrajectorySpeed.NORMAL;

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final IMU imu;

    // Starting heading for imu heading correction, in RADIANS
    private final double startHeading;

    public final Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
            lastLeftBackPos = leftBack.getPositionAndVelocity().position;
            lastRightBackPos = rightBack.getPositionAndVelocity().position;
            lastRightFrontPos = rightFront.getPositionAndVelocity().position;

            lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;
        this.startHeading = pose.heading.toDouble();

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This is to prevent the rr bug of when the motor mode is set to RUN_TO_POSITION and there is no target position set
        leftFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightBack.setTargetPosition(0);
        rightFront.setTargetPosition(0);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

//        imu = null;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

//        localizer = new DriveLocalizer();
        localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);
//        localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);

//        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void correctHeadingWithIMU() {
        double newHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + startHeading;
        pose = new Pose2d(pose.position.x, pose.position.y, newHeading);
    }

    public Action correctHeadingWithIMUAction() {
        return packet -> {
            correctHeadingWithIMU();
            return false;
        };
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public void setDrivePowersForSeconds(PoseVelocity2d powers, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < seconds) {
            setDrivePowers(powers);
            updatePoseEstimate();
        }
        setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.log()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return actionBuilder(beginPose, defaultTrajectorySpeed);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose, TrajectorySpeed speed) {
        SpeedConstraint speedConstraint = defaultSpeedConstraint;
        switch (speed) {
            case SLOW:
                speedConstraint = slowSpeedConstraint;
                break;
            case FAST:
                speedConstraint = fastSpeedConstraint;
                break;
            case ALIGN:
                speedConstraint = alignSpeedConstraint;
                break;
        }
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                speedConstraint.turnConstraints,
                speedConstraint.velConstraint, speedConstraint.accelConstraint,
                0.25, 0.1
        );
    }

    public SpeedConstraint getSpeedConstraint(TrajectorySpeed speed) {
        switch (speed) {
            case SLOW:
                return slowSpeedConstraint;
            case FAST:
                return fastSpeedConstraint;
            default:
                return defaultSpeedConstraint;
        }
    }

    public void drawFieldOverlay() {
        double x = pose.position.x;
        double y = pose.position.y;
        double heading = pose.heading.log();
        double halfWidth = 6;
        double halfLength = 7;
        double[] xPoints = new double[4];
        double[] yPoints = new double[4];
        xPoints[0] = x + halfLength * Math.cos(heading) - halfWidth * Math.sin(heading);
        yPoints[0] = y + halfLength * Math.sin(heading) + halfWidth * Math.cos(heading);
        xPoints[1] = x + halfLength * Math.cos(heading) + halfWidth * Math.sin(heading);
        yPoints[1] = y + halfLength * Math.sin(heading) - halfWidth * Math.cos(heading);
        xPoints[2] = x - halfLength * Math.cos(heading) + halfWidth * Math.sin(heading);
        yPoints[2] = y - halfLength * Math.sin(heading) - halfWidth * Math.cos(heading);
        xPoints[3] = x - halfLength * Math.cos(heading) - halfWidth * Math.sin(heading);
        yPoints[3] = y - halfLength * Math.sin(heading) + halfWidth * Math.cos(heading);
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.strokePolygon(xPoints, yPoints);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}