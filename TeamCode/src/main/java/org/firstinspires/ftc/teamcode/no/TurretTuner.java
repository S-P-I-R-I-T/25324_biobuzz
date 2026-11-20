package org.firstinspires.ftc.teamcode.no;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KLA.Util.Alliance;
import org.firstinspires.ftc.teamcode.KLA.Util.TurretAim;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * 터렛 PID 튜닝/테스트. Panels에서 TurretAim.kP/kI/kD/kS 바꾸면 바로 반영.
 *
 * <pre>
 *   gamepad1 A        스텝 (0° ↔ STEP_DEG 왕복)
 *   gamepad1 B        필드 조준 (BLUE 셀, 로봇 밀어서 추종 확인)
 *   gamepad1 X        수동 (왼쪽 스틱 X로 목표 각도 이동)
 *   D-pad 상/하       스텝 각도 ± 15°
 * </pre>
 */
@Configurable
@Disabled
@TeleOp(name = "Turret Tuner", group = "Tuning")
public class TurretTuner extends OpMode {
    public static double STEP_DEG = 45;
    public static double STEP_PERIOD_S = 2.0;
    public static double MANUAL_DEG_PER_S = 90;
    public static double START_X = 133.5, START_Y = 9.5, START_HEADING_DEG = 90;

    private enum Mode { STEP, FIELD, MANUAL }

    private Localizer localizer;
    private TurretAim aim;
    private TelemetryManager panels;
    private final ElapsedTime stepTimer = new ElapsedTime(), loopTimer = new ElapsedTime();
    private Mode mode = Mode.STEP;
    private boolean high;
    private double manualDeg;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class))
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        TurretAim.resetEncoder(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        aim = new TurretAim(hardwareMap, localizer, Alliance.BLUE);
        aim.stop();
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addLine("터렛을 로봇 정면에 맞춘 뒤 init 하세요");
    }

    @Override
    public void start() {
        stepTimer.reset();
        loopTimer.reset();
        aim.setManualTarget(0.0);
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (gamepad1.aWasPressed()) { mode = Mode.STEP; stepTimer.reset(); }
        if (gamepad1.bWasPressed()) mode = Mode.FIELD;
        if (gamepad1.xWasPressed()) { mode = Mode.MANUAL; manualDeg = aim.currentDeg(); }
        if (gamepad1.dpadUpWasPressed()) STEP_DEG += 15;
        if (gamepad1.dpadDownWasPressed()) STEP_DEG -= 15;

        switch (mode) {
            case STEP:
                if (stepTimer.seconds() >= STEP_PERIOD_S) { high = !high; stepTimer.reset(); }
                aim.setManualTarget(high ? STEP_DEG : 0.0);
                break;
            case FIELD:
                aim.setManualTarget(null);
                break;
            case MANUAL:
                manualDeg -= gamepad1.left_stick_x * MANUAL_DEG_PER_S * dt;
                manualDeg = Math.max(-TurretAim.MAX_ANGLE_DEG, Math.min(TurretAim.MAX_ANGLE_DEG, manualDeg));
                aim.setManualTarget(manualDeg);
                break;
        }

        localizer.update();
        aim.periodic();

        panels.addData("target", aim.targetTicks());
        panels.addData("current", aim.currentTicks());
        panels.addData("error", aim.targetTicks() - aim.currentTicks());
        panels.addData("mode", mode == Mode.FIELD ? "FIELD (" + aim.targetName() + ")" : mode.name());
        panels.addData("power", aim.power());
        panels.addData("gains", String.format("kP %.4f  kI %.4f  kD %.4f  kS %.3f",
                TurretAim.kP, TurretAim.kI, TurretAim.kD, TurretAim.kS));
        Pose p = localizer.pose();
        panels.addData("pose", String.format("x %.1f  y %.1f  h %.1f°", p.x(), p.y(), Math.toDegrees(p.heading())));
        panels.update(telemetry);
    }

    @Override
    public void stop() { aim.stop(); }
}
