package org.firstinspires.ftc.teamcode.no;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KLA.Util.Action;
import org.firstinspires.ftc.teamcode.KLA.Util.Alliance;
import org.firstinspires.ftc.teamcode.KLA.Util.TurretAim;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@Disabled
@Autonomous(name = "BlueFlowerIntakeTest")
public class BlueFlowerIntakeTest extends LinearOpMode {
    public static double SHOOT_TIME_MS = 1500;
    public static double INTAKE_WAIT_MS = 4000;

    private Follower follower;
    private Action action;
    private TurretAim turret;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(83.3, 132.3, 270);
    private final Pose path1 = poseFactory.of(127, 94.5, 0);
    private final Pose point2 = poseFactory.of(128.3, 94.5, 0);
    private final Pose point3 = poseFactory.of(125.4976, 29.9634, 0);
    private final Pose point3Control1 = poseFactory.of(105.8222, 94.4417, 0);
    private final Pose point3Segment1Start = poseFactory.of(125.4976, 29.9634, 0);
    private final Pose point3Segment1End = poseFactory.of(125.4976, 29.9634, 0);

    public Command autoRoutine() {
        return sequential(
                action.spinUp(),
                waitUntil(action::shooterAtSpeed),
                action.feedOn(),
                deadline(waitMs(SHOOT_TIME_MS), action.shoot()),
                action.intakeStop(),
                action.flowerOpen(),
                action.intakeOn(),
                follow(follower, path1()),
                follow(follower, path2()),
                waitMs(INTAKE_WAIT_MS),
                action.intakeStop(),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        action = new Action(hardwareMap);
        action.setDistanceSupplier(() -> Alliance.BLUE.distanceToTarget(follower.pose()));

        TurretAim.resetEncoder(hardwareMap);
        turret = new TurretAim(hardwareMap, null, Alliance.BLUE);   // 오토: 0°(시작 방향) 고정
        turret.stop();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            Scheduler.execute();
            action.periodic();
            turret.periodic();

            telemetry.addData("shooter", "%.0f / %.0f  %s", action.shooterVelocity(), action.targetVelocity(),
                    action.shooterAtSpeed() ? "READY" : "");
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("turret", "%.1f°", turret.currentDeg());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        turret.stop();
        Scheduler.reset();
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(1, Interpolator.linear(point3Segment1Start, point3Segment1End)));
    }
}
