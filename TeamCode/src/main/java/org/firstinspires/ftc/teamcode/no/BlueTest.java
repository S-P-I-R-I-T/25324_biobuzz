package org.firstinspires.ftc.teamcode.no;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
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
@Autonomous
public class BlueTest extends LinearOpMode {
    public static double SHOOT_TIME_MS = 3000;

    private Follower follower;
    private Action action;
    private TurretAim turret;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(83.3, 132.3138, 270);
    private final Pose path1 = poseFactory.of(129.9396, 43.6555, 180);
    private final Pose path1Control1 = poseFactory.of(118.7862, 117.5732, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                action.spinUp(),
                waitUntil(action::shooterAtSpeed),
                action.feedOn(),
                deadline(waitMs(SHOOT_TIME_MS), action.shoot()),
                action.intakeStop(),
                follow(follower, path1())
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
            telemetry.addData("turret", "%.1f°", turret.currentDeg());

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

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
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
}
