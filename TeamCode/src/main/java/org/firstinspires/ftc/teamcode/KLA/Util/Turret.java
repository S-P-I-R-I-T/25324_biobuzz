package org.firstinspires.ftc.teamcode.KLA.Util;

import com.pedropathing.math.Pose;

public class Turret {
    public Turret() {}

    public double SHOOTER_ANGLE_TPR = 537.7;
    public double GEAR_RATIO = 135.0 / 89; // 135/89 는 정수 나눗셈이라 1이 된다

    public double RED_Garden_Cell_X = 58;
    public double RED_Garden_Cell_Y = 54;

    public double RED_Flower_Cell_X = 58;
    public double RED_Flower_Cell_Y = 87.5;

    public double BLUE_Garden_Cell_X = 84.5;
    public double BLUE_Garden_Cell_Y = 87.5;

    public double BLUE_Flower_Cell_X = 84.5;
    public double BLUE_Flower_Cell_Y = 54;

    public int alignTarget(Pose botPose , Pose targetPose) {
        double dy = targetPose.y() - botPose.y();
        double dx = targetPose.x() - botPose.x();

        double targetRad = (Math.atan2(dy, dx) - botPose.heading());

        return RadToTicks(targetRad);
    }

    public double getTargetHeading(Pose botPose, Pose targetPose) {
        double dy = targetPose.y() - botPose.y();
        double dx = targetPose.x() - botPose.x();

        return Math.toDegrees(Math.atan2(dy, dx) - botPose.heading());
    }

    int RadToTicks(double Rad){
        while (Rad > Math.PI) {
            Rad -= 2 * Math.PI;
        }
        while (Rad < -Math.PI) {
            Rad += 2 * Math.PI;
        }

        double ticks = (Rad / (2 * Math.PI)) * SHOOTER_ANGLE_TPR * GEAR_RATIO;

        return (int) ticks;
    }

}