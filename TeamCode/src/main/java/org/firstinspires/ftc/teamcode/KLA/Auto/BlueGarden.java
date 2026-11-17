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
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@Autonomous
public class BlueGarden extends LinearOpMode {
    /** shootFlower 도착 후 발사(급탄) 시간. 슈터는 이동 중 미리 돌려 두므로 거의 전부 급탄 시간이다. */
    public static double SHOOT_TIME_MS = 3000;

    private Follower follower;
    private Action action;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86.6769, 132.4581, 270);
    private final Pose goGarden = poseFactory.of(134, 120, 90);
    private final Pose intakeGardenStart = poseFactory.of(134, 120, 90);
    private final Pose intakeGarden = poseFactory.of(134, 133, 90);
    private final Pose backGarden = poseFactory.of(134, 120, 90);
    private final Pose intakeGardenInsurance = poseFactory.of(134, 133, 90);
    private final Pose shootFlowerStart = poseFactory.of(134, 133, 90);
    private final Pose shootFlower = poseFactory.of(86.8718, 13.7487, 90);
    private final Pose shootFlowerControl1 = poseFactory.of(112.7142, 64.702, 0);
    private final Pose shootFlowerControl2 = poseFactory.of(107.9895, 31.2694, 0);
    private final Pose goFlowerStart = poseFactory.of(86.8718, 13.7487, 90);
    private final Pose goFlower = poseFactory.of(94.3391, 21.628, 270);
    private final Pose intakeFlowerStart = poseFactory.of(94.3391, 21.628, 270);
    private final Pose intakeFlower = poseFactory.of(94.4037, 18.0049, 270);
    private final Pose parkingStart = poseFactory.of(94.4037, 18.0049, 90);
    private final Pose parking = poseFactory.of(127.6825, 24.3707, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, goGarden()),

                // Garden 수집: intakeGarden 시작 ~ intakeGardenInsurance 끝까지 인테이크 ON
                action.intakeOn(),
                follow(follower, intakeGarden()),
                follow(follower, backGarden()),
                follow(follower, intakeGardenInsurance()),
                action.intakeStop(),

                // shootFlower로 가는 동안 슈터 예열 → 도착하면 바로 급탄
                action.spinUp(),
                follow(follower, shootFlower()),
                hold(follower, shootFlower),                         // 발사 중 밀려도 자리 유지
                deadline(waitMs(SHOOT_TIME_MS), action.shoot()),    // 시간 끝나면 shoot 종료 → 슈터 정지, 블로커 닫힘

                // Flower 수집: 플라워 열고 인테이크 ON
                action.flowerOpen(),
                action.intakeOn(),
                follow(follower, goFlower()),
                follow(follower, intakeFlower()),

                // 주차: 인테이크 끄고 플라워 닫기
                action.intakeStop(),
                action.flowerClose(),
                follow(follower, parking())
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
        action = new Action(hardwareMap);   // 플라워 init / 블로커 닫힘 위치로 이동

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            Scheduler.execute();
            action.periodic();   // NextOpMode가 아니므로 직접 호출 (PIDF 전압 보정 + 인테이크/슈터 출력)

            telemetry.addData("shooter", "%.0f / %.0f", action.shooterVelocity(), Action.SHOOTING_VELOCITY);
            telemetry.addData("feed", action.isFeeding() ? "급탄 중 (" + action.feedReason() + ")" : "-");

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        Scheduler.reset();
    }

    public Path goGarden() {
        return Paths.line(start, goGarden).linear(start, goGarden);
    }

    public Path intakeGarden() {
        return Paths.line(intakeGardenStart, intakeGarden).linear(intakeGardenStart, intakeGarden);
    }

    public Path backGarden() {
        return Paths.line(intakeGarden, backGarden).reverseTangent();
    }

    public Path intakeGardenInsurance() {
        return Paths.line(backGarden, intakeGardenInsurance).tangent();
    }

    public Path shootFlower() {
        return Paths.curve(shootFlowerStart, shootFlowerControl1, shootFlowerControl2, shootFlower).linear(shootFlowerStart, shootFlower);
    }

    public Path goFlower() {
        return Paths.line(goFlowerStart, goFlower).linear(goFlowerStart, goFlower);
    }

    public Path intakeFlower() {
        return Paths.line(intakeFlowerStart, intakeFlower).linear(intakeFlowerStart, intakeFlower);
    }

    public Path parking() {
        return Paths.line(parkingStart, parking).linear(parkingStart, parking);
    }
}
