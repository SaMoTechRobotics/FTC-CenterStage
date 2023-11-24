package org.firstinspires.ftc.teamcode.Util.Classes.Lib;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class FieldOverlayUtils {
    public static double PixelWidth = 3.5;
    public static double RobotWidth = 13.0;
    public static double RobotLength = 15.0;

    public static void drawChassis(Canvas canvas, Pose2d position) {
        double x = position.position.x;
        double y = position.position.y;
        double heading = position.heading.log();
        double halfWidth = RobotWidth / 2;
        double halfLength = RobotLength / 2;
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
        canvas.setStroke("blue");
        canvas.strokePolygon(xPoints, yPoints);
    }

    public static void drawPixel(Canvas canvas, Vector2d position) {
        canvas.setFill("white");
        canvas.fillCircle(position.x, position.y, PixelWidth);
        canvas.setStroke("black");
        canvas.strokeCircle(position.x, position.y, PixelWidth);
    }
}
