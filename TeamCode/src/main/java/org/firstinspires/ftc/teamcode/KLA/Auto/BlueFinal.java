package org.firstinspires.ftc.teamcode.KLA.Auto;

import static com.pedropathing.api.Paths.*;
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
import static com.pedropathing.ivy.pedro.PedroCommands.hold;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KLA.Util.Action;
import org.firstinspires.ftc.teamcode.KLA.Util.Alliance;
import org.firstinspires.ftc.teamcode.KLA.Util.HubLed;
import org.firstinspires.ftc.teamcode.KLA.Util.PoseStore;
import org.firstinspires.ftc.teamcode.KLA.Util.TurretAim;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@Autonomous
public class BlueFinal extends LinearOpMode {
    public static double SHOOT_TIME_MS = 2000;
    public static double SERVO_SETTLE_MS = 300;
    public static double INTAKE_WAIT_MS = 3000;

    private Follower follower;
    private Action action;
    private TurretAim turret;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(83.3, 132.3, 270);
    private final Pose shotgointake = poseFactory.of(126, 94.5, 0);
    private final Pose flowerintake = poseFactory.of(128.25, 94.5, 0);
    private final Pose goshotStart = poseFactory.of(128.3, 94.5, 0);
    private final Pose goshot = poseFactory.of(85.4573, 14.4264, 90);
    private final Pose goshotControl1 = poseFactory.of(109.3324, 30.2222, 0);
    private final Pose gointakeStart = poseFactory.of(85.4573, 14.4264, 90);
    private final Pose gointake = poseFactory.of(94.5, 18, 270);
    private final Pose gointakeControl1 = poseFactory.of(87.4256, 27.4404, 0);
    private final Pose flowerintake_2Start = poseFactory.of(94.5, 18, 270);
    private final Pose flowerintake_2 = poseFactory.of(94.5, 16.2, 270);
    private final Pose goshot_2Start = poseFactory.of(94.5, 16.2, 270);
    private final Pose goshot_2 = poseFactory.of(85.46, 14.43, 90);
    private final Pose goshot_2Control1 = poseFactory.of(91.3523, 33.2888, 0);
    private final Pose parking = poseFactory.of(127.1612, 24.1857, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // 0. 서보가 초기 위치에 도착할 때까지 대기
                waitMs(SERVO_SETTLE_MS),

                // 1. 시작 위치에서 발사
                shootHere(),

                // 2. Flower 수집
                action.flowerOpen(),
                action.intakeOn(),
                follow(follower, shotgointake()),
                follow(follower, flowerintake()),
                waitMs(INTAKE_WAIT_MS),
                action.intakeStop(),

                // 3. 이동하면서 예열 → 발사
                action.spinUp(),
                follow(follower, goshot()),
                hold(follower, goshot),
                shootHere(),

                // 4. Flower 수집 2
                action.intakeOn(),
                follow(follower, gointake()),
                follow(follower, flowerintake_2()),
                waitMs(INTAKE_WAIT_MS),
                action.intakeStop(),

                // 5. 이동하면서 예열 → 발사
                action.spinUp(),
                follow(follower, goshot_2()),
                hold(follower, goshot_2),
                shootHere(),
                action.flowerClose(),

                // 6. 주차
                follow(follower, parking())
        );
    }

    /** 슈터 목표 속도 도달 → 스토퍼 열고 급탄 SHOOT_TIME_MS → 슈터/인테이크 정지, 스토퍼 닫힘. */
    private Command shootHere() {
        return sequential(
                action.spinUp(),
                waitUntil(action::shooterAtSpeed),
                action.feedOn(),
                deadline(waitMs(SHOOT_TIME_MS), action.shoot()),
                action.intakeStop()
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
        TurretAim.resetEncoder(hardwareMap);
        HubLed.rainbow(hardwareMap);
        turret = new TurretAim(hardwareMap, null, Alliance.BLUE);   // 오토: 0°(시작 방향) 고정
        turret.stop();

        waitForStart();
        // START 후에 서보 초기 위치로 (init 때는 안 움직임)
        Action.AUTO_SHOT = true;   // TeleOp에서 L3로 MANUAL 해둔 게 남아 있을 수 있음
        action = new Action(hardwareMap);
        action.setDistanceSupplier(() -> Alliance.BLUE.distanceToTarget(follower.pose()));
        schedule(autoRoutine());

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            PoseStore.save(follower.pose(), Alliance.BLUE);   // TeleOp 시작 위치로 넘김
            Scheduler.execute();
            action.periodic();
            turret.periodic();

            telemetry.addData("shooter", "%.0f / %.0f  %s  dist %.1f", action.shooterVelocity(), action.targetVelocity(),
                    action.shooterAtSpeed() ? "READY" : "", action.distance());
            telemetry.addData("hood", "%s  base %.3f  cmd %.3f", action.autoShotActive() ? "AUTO" : "MANUAL",
                    action.baseHood(), action.hoodCommand());

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
        HubLed.restore(hardwareMap);
    }

    public Path shotgointake() {
        return Paths.line(start, shotgointake).linear(start, shotgointake);
    }

    public Path flowerintake() {
        return Paths.line(shotgointake, flowerintake).tangent();
    }

    public Path goshot() {
        return Paths.curve(goshotStart, goshotControl1, goshot).linear(goshotStart, goshot);
    }

    public Path gointake() {
        return Paths.curve(gointakeStart, gointakeControl1, gointake).linear(gointakeStart, gointake);
    }

    public Path flowerintake_2() {
        return Paths.line(flowerintake_2Start, flowerintake_2).linear(flowerintake_2Start, flowerintake_2);
    }

    public Path goshot_2() {
        return Paths.curve(goshot_2Start, goshot_2Control1, goshot_2).linear(goshot_2Start, goshot_2);
    }

    public Path parking() {
        return Paths.line(goshot_2, parking).linear(goshot_2, parking);
    }
}
