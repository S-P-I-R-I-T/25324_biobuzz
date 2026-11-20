package org.firstinspires.ftc.teamcode.no;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KLA.Util.Action;

/**
 * 슈터 속도 PIDF 튜너 (gamepad1).
 *
 * <pre>
 *   A            슈터 ON / OFF
 *   LB / RB      목표 속도 - / + (TARGET_STEP)
 *   D-Pad 좌/우   조정할 계수 선택 (P, I, D, F)
 *   D-Pad 상/하   선택한 계수 + / - (스텝 크기만큼)
 *   B            스텝 크기 변경
 *   RT (누르는 동안) 블로커 열고 인테이크 풀 속도 → 발사 중 속도 회복 확인
 *
 *   Y            자동 F 측정: 풀 파워로 돌려 최고 속도를 재고 F를 계산한다.
 *   X            자동 P 탐색: 스텝 응답을 반복 측정하며 P를 찾는다.
 *   (자동 실행 중 A / X / Y 아무거나 누르면 취소)
 * </pre>
 *
 * 권장 순서: Y(자동 F) → X(자동 P) → 수동 미세 조정 + RT로 발사 중 속도 저하 확인.
 * 시작 값은 Action의 현재 값이다. 튜닝이 끝나면 텔레메트리의 값을 Action.java에 옮겨 적는다.
 * F는 Action과 동일하게 배터리 전압(12V 기준)으로 보정해서 적용한다.
 */
@Configurable
@Disabled
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterPIDFTuner extends OpMode {
    public static double P = Action.P, I = Action.I, D = Action.D, F = Action.F;
    public static double TARGET_VELOCITY = Action.SHOOTING_VELOCITY;
    public static double TARGET_STEP = 50;
    /** 이 오차(ticks/s) 안에 들어오면 "도달"로 본다. */
    public static double TOLERANCE = Action.SHOOTER_VELOCITY_TOLERANCE;
    /** 목표 대비 이 비율(%)을 넘는 오버슈트는 "과하다"고 본다. */
    public static double OVERSHOOT_LIMIT_PERCENT = 3.0;

    // 자동 F 측정
    public static double AUTO_F_RUN_SEC = 3.0;

    // 자동 P 탐색: [REST: 목표의 절반으로 감속] → [STEP: 목표로 가속하며 측정] 을 반복
    public static double AUTO_P_REST_SEC = 1.5;
    public static double AUTO_P_STEP_SEC = 2.5;
    public static double AUTO_P_SETTLE_WINDOW_SEC = 0.7; // STEP 마지막 구간에서 진동을 본다
    public static double AUTO_P_INCREASE = 1.3;
    public static double AUTO_P_DECREASE = 0.75;
    public static int AUTO_P_MAX_TRIALS = 10;

    private enum Mode { MANUAL, AUTO_F, AUTO_P }

    private static final String[] GAIN_NAMES = {"P", "I", "D", "F"};
    private static final double[] STEP_SIZES = {10.0, 1.0, 0.1, 0.01, 0.001};
    private static final double VOLTAGE_REFRESH_SEC = 0.5;
    private static final double HINT_DELAY_SEC = 2.5;

    private DcMotorEx shooter;
    private DcMotor intake;
    private Servo blocker;

    private Mode mode = Mode.MANUAL;
    private int selectedGain = 0;
    private int stepIndex = 1;
    private boolean shooterOn = false;
    private String autoResult = "";

    private double batteryVoltage = 12.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private PIDFCoefficients applied;

    // 수동 모드 응답 측정: 목표가 바뀔 때마다 초기화된다.
    private final ElapsedTime responseTimer = new ElapsedTime();
    private double spinUpSec = Double.NaN;   // 목표 도달까지 걸린 시간
    private double peakVelocity = 0;         // 최고 속도 (오버슈트 확인)
    private double worstErrorAfterReach = 0; // 도달 이후 최대 오차 (발사 시 속도 저하 확인)

    // 자동 모드 상태
    private final ElapsedTime autoTimer = new ElapsedTime();
    private double autoFMaxVelocity;
    private boolean autoPStepPhase;
    private int autoPTrial;
    private boolean autoPDecreasing;
    private double autoPStartP, autoPLastGoodP;
    private double trialPeak, trialReachSec, trialWindowMin, trialWindowMax;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blocker.setPosition(Action.blocker_close);

        batteryVoltage = readBatteryVoltage();
        applyPidf();

        telemetry.addLine("Shooter PIDF Tuner 준비 완료");
        telemetry.addLine("A: 슈터 ON/OFF   Y: 자동 F   X: 자동 P");
    }

    @Override
    public void loop() {
        if (voltageTimer.seconds() >= VOLTAGE_REFRESH_SEC) {
            batteryVoltage = readBatteryVoltage();
            voltageTimer.reset();
        }

        double velocity = shooter.getVelocity();
        switch (mode) {
            case AUTO_F: runAutoF(velocity); break;
            case AUTO_P: runAutoP(velocity); break;
            default: runManual(velocity); break;
        }
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        blocker.setPosition(Action.blocker_close);
    }

    // ───────────────────────── 수동 모드 ─────────────────────────

    private void runManual(double velocity) {
        handleManualInput();
        if (mode != Mode.MANUAL) return; // 방금 자동 모드를 시작했다

        applyPidf();
        double target = shooterOn ? TARGET_VELOCITY : 0;
        shooter.setVelocity(target);

        boolean feeding = gamepad1.right_trigger > 0.5;
        setFeeding(feeding);

        double error = target - velocity;
        boolean atSpeed = shooterOn && Math.abs(error) <= TOLERANCE;
        updateResponseStats(velocity, error, atSpeed);

        telemetry.addData("모드", "수동%s", feeding ? "  (급송 중)" : "");
        telemetry.addData("슈터", shooterOn ? "ON" : "OFF");
        addVelocityTelemetry(target, velocity);

        telemetry.addLine();
        telemetry.addLine("── 응답 ──");
        telemetry.addData("도달 시간", Double.isNaN(spinUpSec) ? "-" : String.format("%.2f s", spinUpSec));
        telemetry.addData("오버슈트", "%.0f", Math.max(0, peakVelocity - TARGET_VELOCITY));
        telemetry.addData("도달 후 최대 오차", "%.0f", worstErrorAfterReach);
        telemetry.addData("힌트", tuningHint(error));

        addGainTelemetry();
        if (!autoResult.isEmpty()) {
            telemetry.addLine();
            telemetry.addData("자동 튜닝 결과", autoResult);
        }
    }

    private void handleManualInput() {
        if (gamepad1.yWasPressed()) { startAutoF(); return; }
        if (gamepad1.xWasPressed()) { startAutoP(); return; }

        if (gamepad1.aWasPressed()) {
            shooterOn = !shooterOn;
            resetResponseStats();
        }
        if (gamepad1.rightBumperWasPressed()) {
            TARGET_VELOCITY += TARGET_STEP;
            resetResponseStats();
        }
        if (gamepad1.leftBumperWasPressed()) {
            TARGET_VELOCITY = Math.max(0, TARGET_VELOCITY - TARGET_STEP);
            resetResponseStats();
        }

        if (gamepad1.dpadRightWasPressed()) selectedGain = (selectedGain + 1) % GAIN_NAMES.length;
        if (gamepad1.dpadLeftWasPressed()) selectedGain = (selectedGain + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
        if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % STEP_SIZES.length;

        if (gamepad1.dpadUpWasPressed()) adjustSelectedGain(+STEP_SIZES[stepIndex]);
        if (gamepad1.dpadDownWasPressed()) adjustSelectedGain(-STEP_SIZES[stepIndex]);
    }

    private void adjustSelectedGain(double delta) {
        switch (selectedGain) {
            case 0: P = Math.max(0, P + delta); break;
            case 1: I = Math.max(0, I + delta); break;
            case 2: D = Math.max(0, D + delta); break;
            default: F = Math.max(0, F + delta); break;
        }
    }

    private void resetResponseStats() {
        responseTimer.reset();
        spinUpSec = Double.NaN;
        peakVelocity = 0;
        worstErrorAfterReach = 0;
    }

    private void updateResponseStats(double velocity, double error, boolean atSpeed) {
        if (!shooterOn) return;
        peakVelocity = Math.max(peakVelocity, velocity);
        if (Double.isNaN(spinUpSec)) {
            if (atSpeed) spinUpSec = responseTimer.seconds();
        } else {
            worstErrorAfterReach = Math.max(worstErrorAfterReach, Math.abs(error));
        }
    }

    /** 지금까지의 응답을 보고 어느 계수를 어느 쪽으로 움직일지 알려준다. */
    private String tuningHint(double error) {
        if (!shooterOn) return "A로 슈터를 켜세요 (처음이면 Y → X 순서 추천)";
        if (responseTimer.seconds() < HINT_DELAY_SEC) return "측정 중...";

        if (Double.isNaN(spinUpSec)) {
            return error > 0
                    ? "목표에 못 미침 → F를 올리세요 (Y로 자동 측정)"
                    : "목표보다 빠름 → F를 내리세요 (Y로 자동 측정)";
        }
        if (peakVelocity - TARGET_VELOCITY > overshootLimit()) {
            return "오버슈트 큼 → P를 낮추세요 (그래도 크면 D를 조금 추가)";
        }
        if (worstErrorAfterReach > 2 * TOLERANCE) {
            return "도달 후 속도가 흔들림 → 가만히 둬도 진동하면 P↓, 발사할 때만 떨어지면 P↑";
        }
        if (spinUpSec > 1.5) return "안정적이지만 느림 → P를 올리세요";
        return "양호 — RT로 급송하며 속도 저하를 확인하세요";
    }

    // ───────────────────────── 자동 F 측정 ─────────────────────────
    // 풀 파워에서의 최고 속도로 F = 32767 / maxVelocity 를 구한다 (REV 허브 velocity PIDF 기준).
    // Action이 F * (12 / 전압)으로 보정하므로, 측정 당시 전압을 반대로 곱해 12V 기준 값으로 저장한다.

    private void startAutoF() {
        mode = Mode.AUTO_F;
        shooterOn = false;
        setFeeding(false);
        autoFMaxVelocity = 0;
        autoTimer.reset();
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(1.0);
    }

    private void runAutoF(double velocity) {
        if (cancelPressed()) {
            finishAuto("자동 F 취소됨 (값 변경 없음)");
            return;
        }
        autoFMaxVelocity = Math.max(autoFMaxVelocity, velocity);

        if (autoTimer.seconds() >= AUTO_F_RUN_SEC) {
            if (autoFMaxVelocity < 100 || batteryVoltage <= 0) {
                finishAuto("자동 F 실패: 속도가 측정되지 않음 (엔코더 케이블 / 모터 방향 확인)");
            } else {
                F = 32767.0 / autoFMaxVelocity * (batteryVoltage / 12.0);
                finishAuto(String.format("F = %.3f  (최고 속도 %.0f, %.2f V)", F, autoFMaxVelocity, batteryVoltage));
            }
            return;
        }

        telemetry.addData("모드", "자동 F 측정 중  (A/X/Y: 취소)");
        telemetry.addData("진행", "%.1f / %.1f s", autoTimer.seconds(), AUTO_F_RUN_SEC);
        telemetry.addData("현재 속도", "%.0f", velocity);
        telemetry.addData("최고 속도", "%.0f", autoFMaxVelocity);
        telemetry.addData("배터리", "%.2f V", batteryVoltage);
    }

    // ───────────────────────── 자동 P 탐색 ─────────────────────────
    // 매 시도마다 목표의 절반 → 목표로 스텝을 주고 오버슈트와 진동을 측정한다.
    //   첫 시도가 양호: 불량이 나올 때까지 P를 키우고, 마지막으로 양호했던 P로 확정한다.
    //   첫 시도가 불량: 양호해질 때까지 P를 줄이고, 처음 양호해진 P로 확정한다.

    private void startAutoP() {
        mode = Mode.AUTO_P;
        shooterOn = false;
        setFeeding(false);
        autoPStartP = P;
        autoPLastGoodP = Double.NaN;
        autoPDecreasing = false;
        autoPTrial = 1;
        if (P <= 0) P = 1.0; // 0에서는 곱해서 키울 수 없다
        beginAutoPPhase(false);
    }

    private void beginAutoPPhase(boolean stepPhase) {
        autoPStepPhase = stepPhase;
        autoTimer.reset();
        trialPeak = 0;
        trialReachSec = Double.NaN;
        trialWindowMin = Double.POSITIVE_INFINITY;
        trialWindowMax = Double.NEGATIVE_INFINITY;
    }

    private void runAutoP(double velocity) {
        if (cancelPressed()) {
            P = autoPStartP;
            finishAuto("자동 P 취소됨 (P 원래 값으로 복구)");
            return;
        }

        applyPidf();
        double target = autoPStepPhase ? TARGET_VELOCITY : TARGET_VELOCITY * 0.5;
        shooter.setVelocity(target);
        double elapsed = autoTimer.seconds();

        if (!autoPStepPhase) {
            if (elapsed >= AUTO_P_REST_SEC) beginAutoPPhase(true);
        } else {
            trialPeak = Math.max(trialPeak, velocity);
            if (Double.isNaN(trialReachSec) && Math.abs(target - velocity) <= TOLERANCE) trialReachSec = elapsed;
            if (elapsed >= AUTO_P_STEP_SEC - AUTO_P_SETTLE_WINDOW_SEC) {
                trialWindowMin = Math.min(trialWindowMin, velocity);
                trialWindowMax = Math.max(trialWindowMax, velocity);
            }
            if (elapsed >= AUTO_P_STEP_SEC && evaluateAutoPTrial()) return;
        }

        telemetry.addData("모드", "자동 P 탐색 중  (A/X/Y: 취소)");
        telemetry.addData("시도", "%d / %d  (%s)", autoPTrial, AUTO_P_MAX_TRIALS,
                autoPStepPhase ? "가속 측정" : "감속 대기");
        telemetry.addData("시험 중인 P", "%.3f  (%s)", P, autoPDecreasing ? "줄이는 중" : "키우는 중");
        telemetry.addData("마지막 양호 P", Double.isNaN(autoPLastGoodP) ? "-" : String.format("%.3f", autoPLastGoodP));
        addVelocityTelemetry(target, velocity);
    }

    /** 한 번의 스텝 시도를 평가하고 다음 P를 정한다. 탐색이 끝났으면 true. */
    private boolean evaluateAutoPTrial() {
        double overshoot = trialPeak - TARGET_VELOCITY;
        double ripple = trialWindowMax - trialWindowMin;
        boolean bad = overshoot > overshootLimit() || ripple > 2 * TOLERANCE;

        if (autoPTrial == 1) autoPDecreasing = bad;

        if (autoPDecreasing) {
            if (!bad) {
                finishAuto(String.format("P = %.3f  (오버슈트 %.0f, 진동폭 %.0f)", P, Math.max(0, overshoot), ripple));
                return true;
            }
            P *= AUTO_P_DECREASE;
        } else {
            if (bad) {
                P = autoPLastGoodP;
                finishAuto(String.format("P = %.3f  (이보다 키우면 오버슈트/진동 발생)", P));
                return true;
            }
            autoPLastGoodP = P;
            P *= AUTO_P_INCREASE;
        }

        if (++autoPTrial > AUTO_P_MAX_TRIALS) {
            if (!Double.isNaN(autoPLastGoodP)) P = autoPLastGoodP;
            finishAuto(String.format("P = %.3f  (최대 시도 횟수 도달 — F와 목표 속도를 확인하세요)", P));
            return true;
        }
        beginAutoPPhase(false);
        return false;
    }

    private boolean cancelPressed() {
        // 세 버튼 모두 읽어서 눌림 기록을 비운다 (|| 로 묶으면 뒤쪽이 남아 수동 모드에서 오작동한다).
        boolean a = gamepad1.aWasPressed();
        boolean x = gamepad1.xWasPressed();
        boolean y = gamepad1.yWasPressed();
        return a || x || y;
    }

    /** 자동 모드를 끝내고 슈터를 끈 채 수동 모드로 돌아간다. */
    private void finishAuto(String result) {
        autoResult = result;
        mode = Mode.MANUAL;
        shooterOn = false;
        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applied = null; // 모드 전환 후 PIDF를 다시 쓴다
        applyPidf();
        shooter.setVelocity(0);
        resetResponseStats();
    }

    // ───────────────────────── 공통 ─────────────────────────

    private void setFeeding(boolean feeding) {
        blocker.setPosition(feeding ? Action.blocker_open : Action.blocker_close);
        intake.setPower(feeding ? 1.0 : 0);
    }

    private double overshootLimit() {
        return TARGET_VELOCITY * OVERSHOOT_LIMIT_PERCENT / 100.0;
    }

    private double gainValue(int index) {
        switch (index) {
            case 0: return P;
            case 1: return I;
            case 2: return D;
            default: return F;
        }
    }

    private double compensatedF() {
        return batteryVoltage > 0 ? F * (12.0 / batteryVoltage) : F;
    }

    /** 값이 바뀌었을 때만 모터 컨트롤러에 다시 써서 루프 시간을 아낀다. */
    private void applyPidf() {
        double f = compensatedF();
        if (applied != null && applied.p == P && applied.i == I && applied.d == D
                && Math.abs(applied.f - f) < 0.01) {
            return;
        }
        applied = new PIDFCoefficients(P, I, D, f);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, applied);
    }

    private double readBatteryVoltage() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && voltage < lowest) lowest = voltage;
        }
        return Double.isInfinite(lowest) ? 0 : lowest;
    }

    private void addVelocityTelemetry(double target, double velocity) {
        double error = target - velocity;
        telemetry.addData("목표 속도", "%.0f  (LB/RB ±%.0f)", target, TARGET_STEP);
        telemetry.addData("현재 속도", "%.0f", velocity);
        telemetry.addData("오차", "%+.0f  (%.1f%%)  %s", error,
                target > 0 ? Math.abs(error) / target * 100 : 0,
                target > 0 && Math.abs(error) <= TOLERANCE ? "[도달]" : "");
    }

    private void addGainTelemetry() {
        telemetry.addLine();
        telemetry.addLine("── 계수 (D-Pad 좌/우 선택, 상/하 조정) ──");
        for (int i = 0; i < GAIN_NAMES.length; i++) {
            telemetry.addData((i == selectedGain ? "▶ " : "   ") + GAIN_NAMES[i], "%.4f", gainValue(i));
        }
        telemetry.addData("스텝 크기", "%s  (B)", STEP_SIZES[stepIndex]);
        telemetry.addLine("Y: 자동 F 측정   X: 자동 P 탐색");

        telemetry.addLine();
        telemetry.addData("배터리", "%.2f V", batteryVoltage);
        telemetry.addData("적용된 F (전압 보정)", "%.4f", compensatedF());
    }
}
