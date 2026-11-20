package org.firstinspires.ftc.teamcode.KLA.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.KLA.Util.Alliance;
import org.firstinspires.ftc.teamcode.KLA.Util.Robot;

import dev.nextftc.robot.opmode.NextTeleop;

@Configurable
@NextTeleop(name = "Blue")
public class Blue extends AllianceTeleop {
    /** OPTIONS = 시작 위치(90°), SHARE = 위쪽 모서리(270°). */
    public static double OPTIONS_X = 133.5, OPTIONS_Y = 9.5;
    public static double SHARE_X = 133.5, SHARE_Y = 132;
    public static double OPTIONS_HEADING_DEG = 90, SHARE_HEADING_DEG = 270;

    public Blue(Robot robot) { super(robot, Alliance.BLUE); }

    @Override protected Pose optionsPose() { return new Pose(OPTIONS_X, OPTIONS_Y, Math.toRadians(OPTIONS_HEADING_DEG)); }
    @Override protected Pose sharePose() { return new Pose(SHARE_X, SHARE_Y, Math.toRadians(SHARE_HEADING_DEG)); }
}
