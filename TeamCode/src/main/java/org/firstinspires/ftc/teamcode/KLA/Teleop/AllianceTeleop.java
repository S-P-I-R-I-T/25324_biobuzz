package org.firstinspires.ftc.teamcode.KLA.Teleop;

import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.BulkReadHook;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.commands.Commands.instant;
import org.firstinspires.ftc.teamcode.KLA.Util.Action;
import org.firstinspires.ftc.teamcode.KLA.Util.Alliance;
import org.firstinspires.ftc.teamcode.KLA.Util.Drive;
import org.firstinspires.ftc.teamcode.KLA.Util.HubLed;
import org.firstinspires.ftc.teamcode.KLA.Util.PoseStore;
import org.firstinspires.ftc.teamcode.KLA.Util.Robot;
import org.firstinspires.ftc.teamcode.KLA.Util.ShotTable;
import org.firstinspires.ftc.teamcode.KLA.Util.TurretAim;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Blue / Red TeleOp 공통. 진영과 리셋 위치만 하위 클래스에서 정한다. */
@Configurable
public abstract class AllianceTeleop extends NextOpMode {
    private final Alliance ALLIANCE;
    private final Robot robot;
    /** 발사 준비 표시: 터렛 오차 허용(°). */
    public static double READY_AIM_DEG = 2;
    private boolean startedFromAuto, wasReady;
    private Drive drive;
    private Localizer localizer;
    private TurretAim turretAim;

    /** OPTIONS 위치 (= 시작 위치). */
    protected abstract Pose optionsPose();
    /** SHARE 위치. */
    protected abstract Pose sharePose();

    protected AllianceTeleop(Robot robot, Alliance alliance) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        this.ALLIANCE = alliance;
        TurretAim.resetEncoder(hardwareMap);   // init 시점 터렛 방향 = 0°
        HubLed.rainbow(hardwareMap);
    }

    @Override public void start() {
        Scheduler.reset();
        drive = new Drive(hardwareMap, telemetry, gamepad1);
        drive.init();
        robot.action = new Action(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Pose fromAuto = PoseStore.take(ALLIANCE);
        startedFromAuto = fromAuto != null;
        localizer.setPose(startedFromAuto ? fromAuto : optionsPose());
        turretAim = new TurretAim(hardwareMap, localizer, ALLIANCE);
        robot.mechanisms = java.util.Set.of(robot.action, turretAim);
        robot.action.setDistanceSupplier(() -> ALLIANCE.distanceToTarget(localizer.pose()));
        robot.action.setKeepSpinning(true);   // 슈터 상시 예열
        CommandGamepad operator = new CommandGamepad(gamepad2);
        Action action = robot.action;

        // 커맨드가 끝날 때(버튼 해제) 스스로 상태를 해제하므로 onFalse(intakeOff)는 필요 없다.
        operator.a().whileTrue(action.intakeForward());   // 발사 중에도 동작 (발사와 독립)
        operator.b().whileTrue(action.intakeReverse());
        operator.rightTrigger().isOver(0.5)
                .whileTrue(action.shoot())
                .onFalse(action.stopShooter());
        operator.dpadLeft().onTrue(action.blockerOpen());
        operator.dpadRight().onTrue(action.blockerClose());
        operator.dpadUp().onTrue(action.flowerOpen());
        operator.dpadDown().onTrue(action.flowerClose());

        // 후드 값 찾기용: 범퍼 = 미세(±0.01), Y/X = 크게(±0.05). 현재 값은 텔레메트리 "hood"
        operator.rightBumper().onTrue(action.hoodAdjust(Action.HOOD_FINE_STEP));
        operator.leftBumper().onTrue(action.hoodAdjust(-Action.HOOD_FINE_STEP));
        operator.y().onTrue(action.hoodAdjust(Action.HOOD_COARSE_STEP));
        operator.x().onTrue(action.hoodAdjust(-Action.HOOD_COARSE_STEP));

        // 로컬라이저 재설정: 로봇을 해당 모서리에 두고 OPTIONS / SHARE (gamepad1, 2 둘 다)
        for (CommandGamepad pad : new CommandGamepad[]{new CommandGamepad(gamepad1), operator}) {
            pad.start().onTrue(instant(() -> localizer.setPose(optionsPose())));
            pad.back().onTrue(instant(() -> localizer.setPose(sharePose())));
        }
        operator.leftStickButton().onTrue(instant(() -> Action.AUTO_SHOT = !Action.AUTO_SHOT));
        // 표 기록: R3 = 지금 거리 + 수동 후드 추가, 왼쪽 트리거 누른 채 R3 = 마지막 점 삭제
        operator.rightStickButton().onTrue(instant(() -> {
            if (gamepad2.left_trigger > 0.5) ShotTable.removeLast();
            else ShotTable.add(ALLIANCE.distanceToTarget(localizer.pose()), Action.hood_pos);
            telemetry.log().add(ShotTable.code());
        }));
    }

    @Override public void end() {
        HubLed.restore(hardwareMap);
    }

    @Override public void periodic() {
        if (localizer != null) {
            localizer.update();
            Pose pose = localizer.pose();
            telemetry.addData("POSE", "x %.2f  y %.2f  heading %.1f°  (%s)", pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                    startedFromAuto ? "오토에서 이어받음" : "OPTIONS 위치 시작");
        }
        if (drive != null) drive.controlWithPad();
        Action action = robot.action;
        if (action != null) {
            telemetry.addData("shot", "%s  dist %.1f in  (L3 AUTO/MANUAL, R3 기록, LT+R3 지우기)",
                    action.autoShotActive() ? "AUTO" : "MANUAL", action.distance());
            telemetry.addData("table", "%d점  %s", ShotTable.count(), ShotTable.code());
            telemetry.addData("shooter", "%.0f / %.0f  %s",
                    action.shooterVelocity(), action.targetVelocity(),
                    action.shooterAtSpeed() ? "READY" : "spin-up");
            telemetry.addData("hood", "base %.3f  cmd %.3f", action.baseHood(), action.hoodCommand());
        }
        if (action != null && turretAim != null) {
            boolean ready = action.shooterSpunUp() && turretAim.aimed(READY_AIM_DEG);
            if (ready && !wasReady) gamepad2.rumble(200);
            wasReady = ready;
            HubLed.showReady(hardwareMap, ready);
            telemetry.addData("READY", ready ? "■■■ 발사 가능 ■■■" : "-");
        }
        if (turretAim != null && turretAim.pose() != null) {
            telemetry.addData("turret", "%s  엔코더 %d / 목표 %d  (power %.2f)", turretAim.targetName(),
                    turretAim.currentTicks(), turretAim.targetTicks(), turretAim.power());
        }
    }
}
