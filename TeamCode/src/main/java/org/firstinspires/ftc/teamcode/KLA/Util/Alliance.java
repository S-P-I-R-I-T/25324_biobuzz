package org.firstinspires.ftc.teamcode.KLA.Util;

import com.pedropathing.math.Pose;

/** 진영별 셀 위치. 로봇과 같은 쪽(HALF_LINE_Y 기준)에 있는 셀을 목표로 한다. */
public enum Alliance {
    BLUE(new Pose(84.5, 87.5), new Pose(84.5, 54)),
    RED(new Pose(58, 54), new Pose(58, 87.5));

    public static double HALF_LINE_Y = 70.75;

    private final Pose garden, flower;

    Alliance(Pose garden, Pose flower) {
        this.garden = garden;
        this.flower = flower;
    }

    public Pose garden() { return garden; }
    public Pose flower() { return flower; }

    public Pose targetFor(Pose robot) {
        return (robot.y() > HALF_LINE_Y) == (garden.y() > HALF_LINE_Y) ? garden : flower;
    }

    public double distanceToTarget(Pose robot) {
        Pose t = targetFor(robot);
        return Math.hypot(t.x() - robot.x(), t.y() - robot.y());
    }
}

// red 8,9.5 option button red / 8,132 share button red

// 133.5 / 132 share button blue