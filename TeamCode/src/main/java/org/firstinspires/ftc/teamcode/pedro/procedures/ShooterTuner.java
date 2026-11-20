package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Pedro Tuning 웹 UI에서 실행하는 슈터 속도 PIDF 자동 튜너.
 *
 * <ol>
 *   <li>F (피드포워드): 여러 파워로 개루프 회전 → 전압을 12V로 정규화한 파워-속도 직선을 맞추고,
 *       목표 속도에 필요한 파워로 F = 32767 · power / target 을 계산한다.</li>
 *   <li>P: 목표의 60% → 100% 스텝 응답(발사 후 속도 회복과 같은 상황)을 MIN_P부터 2배씩 키우며 측정하고,
 *       오버슈트/진동 한계를 넘지 않는 것 중 가장 빨리 회복하는 P를 고른다.</li>
 *   <li>I: 정상상태 오차가 남으면 I를 조금씩 올려 오차를 없애는 가장 작은 값을 찾는다.</li>
 *   <li>검증: 정지 → 목표, 60% → 목표 응답을 측정해 보고한다.</li>
 * </ol>
 *
 * F는 {@code Action}과 똑같이 12V 기준 값으로 내보낸다(Action이 실제 전압으로 보정해서 적용).
 */
public class ShooterTuner extends Procedure {
    public ShooterTuner() {
        super("Shooter PIDF Tuner", "슈터 플라이휠의 속도 PIDF(P, I, F)를 자동으로 찾는다. 공을 모두 빼고 실행하세요.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs in = inputs("Shooter", "슈터 모터 설정과 목표 성능을 입력하세요.");
        Inputs.Field<String> name = in.s("Motor Name").withDefault("shooter");
        Inputs.Field<Direction> dir = in.e("Motor Direction (Action과 같게)", Direction.class).withDefault(Direction.REVERSE);
        Inputs.Field<Double> target = in.d("Target Velocity (ticks/s)").min(100.0).withDefault(1500.0);
        Inputs.Field<Double> tolerance = in.d("Velocity Tolerance (ticks/s)").min(1.0).withDefault(50.0);
        Inputs.Field<Double> overshoot = in.d("Max Overshoot (%)").min(0.0).withDefault(3.0);
        awaitInputs(in);

        Config cfg = new Config(name.get(), DcMotorSimple.Direction.valueOf(dir.get().name()),
                target.get(), tolerance.get(), overshoot.get());

        confirmation("Safety", "슈터가 풀파워를 포함해 여러 번 회전합니다. 공을 모두 빼고, 손과 케이블을 플라이휠에서 떨어뜨리세요. 전체 약 2분 걸립니다.");

        // 1) F
        FeedforwardResult ff = runOpMode(new FeedforwardId(cfg));
        if (ff == null) abort("중단되었습니다.");
        if (ff.slope <= 0) abort("파워를 올려도 속도가 늘지 않습니다. 엔코더 연결이나 Motor Direction을 확인하세요 (측정 속도가 음수면 방향이 반대입니다).");
        if (ff.maxVelocity12V < cfg.target)
            abort(String.format(Locale.US, "12V 풀파워 최고 속도가 %.0f ticks/s로 목표 %.0f보다 낮습니다. 목표 속도를 낮추세요.",
                    ff.maxVelocity12V, cfg.target));
        double F = ff.f;
        result("Max Velocity @12V (ticks/s)", fmt(ff.maxVelocity12V));
        result("Power for Target @12V", fmt(ff.powerForTarget12V));
        result("F", fmt(F));

        // 2) P
        Double P = runOpMode(new PSearch(cfg, F));
        if (P == null) abort("중단되었습니다.");
        result("P", fmt(P));

        // 3) I
        Double I = runOpMode(new ISearch(cfg, P, F));
        if (I == null) abort("중단되었습니다.");
        result("I", fmt(I));

        // 4) 검증
        Verification v = runOpMode(new Verify(cfg, P, I, F));
        if (v == null) abort("중단되었습니다.");
        result("Spin-up 0 → target (s)", fmt(v.spinUpSec));
        result("Recovery 60% → target (s)", fmt(v.recoverySec));
        result("Overshoot (%)", fmt(v.overshootPercent));
        result("Steady-state Error (ticks/s)", fmt(v.steadyError));
        result("Noise σ (ticks/s)", fmt(v.noise));

        code(Language.JAVA, String.format(Locale.US,
                "// KLA/Util/Action.java\n" +
                "public static double P = %s, I = %s, D = 0.0, F = %s;\n" +
                "public static double SHOOTING_VELOCITY = %s;",
                fmt(P), fmt(I), fmt(F), fmt(cfg.target)));
    }

    static String fmt(double v) {
        return String.format(Locale.US, "%.4f", v).replaceAll("0+$", "").replaceAll("\\.$", ".0");
    }

    // ───────────────────────── 공통 ─────────────────────────

    static final class Config {
        final String name;
        final DcMotorSimple.Direction direction;
        final double target, tolerance, overshootPercent;

        Config(String name, DcMotorSimple.Direction direction, double target, double tolerance, double overshootPercent) {
            this.name = name;
            this.direction = direction;
            this.target = target;
            this.tolerance = tolerance;
            this.overshootPercent = overshootPercent;
        }
    }

    /** 스텝 응답 측정 결과. */
    static final class StepStats {
        double riseSec = Double.NaN;   // target - tolerance 도달 시간
        double peak;                   // 최고 속도
        double windowMean, windowStd;  // 마지막 구간 평균 / 표준편차 (필터된 속도 기준)

        double overshootPercent(double target) { return Math.max(0, (peak - target) / target * 100); }
        double noise() { return windowStd; }
        double steadyError(double target) { return windowMean - target; }
    }

    static final class FeedforwardResult {
        double slope, intercept, maxVelocity12V, powerForTarget12V, f;
    }

    static final class Verification {
        double spinUpSec, recoverySec, overshootPercent, steadyError, noise;
    }

    /** 슈터를 다루는 TuningOpMode 공통 기능. */
    abstract static class ShooterOpMode<R> extends TuningOpMode<R> {
        static final double STEP_LOW_RATIO = 0.6;
        static final double VELOCITY_FILTER_SEC = 0.04;
        final Config cfg;
        DcMotorEx motor;
        String status = "";

        ShooterOpMode(String name, String description, Config cfg) {
            super(name, description, false);
            this.cfg = cfg;
        }

        @Override
        protected final R runTuningOpMode() throws InterruptedException {
            motor = hardwareMap.get(DcMotorEx.class, cfg.name);
            motor.setDirection(cfg.direction);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            waitForStart();
            try {
                return runShooter();
            } finally {
                motor.setPower(0);
            }
        }

        /** null을 반환하면 중단된 것으로 본다. */
        abstract R runShooter() throws InterruptedException;

        boolean active() { return opModeIsActive() && !isStopRequested(); }

        double voltage() {
            double lowest = Double.POSITIVE_INFINITY;
            for (VoltageSensor s : hardwareMap.voltageSensor) {
                double v = s.getVoltage();
                if (v > 0 && v < lowest) lowest = v;
            }
            return Double.isInfinite(lowest) ? 12.0 : lowest;
        }

        /** Action과 같은 방식: F는 12V 기준 값 → 현재 전압으로 보정해서 적용. */
        void applyPidf(double p, double i, double d, double f12) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(p, i, d, f12 * 12.0 / voltage()));
        }

        void show(String line) {
            telemetry.addLine(status);
            telemetry.addLine(line);
            telemetry.update();
        }

        /** 슈터를 세우고 거의 멈출 때까지(최대 5초) 기다린다. */
        void coastToStop() {
            motor.setPower(0);
            long end = System.nanoTime() + 5_000_000_000L;
            while (active() && System.nanoTime() < end && Math.abs(motor.getVelocity()) > 30) {
                show(String.format(Locale.US, "정지 대기  %.0f ticks/s", motor.getVelocity()));
            }
        }

        /** 지금 설정된 상태로 seconds 동안 기다린다(텔레메트리 표시). */
        void hold(double seconds, String label) {
            long end = System.nanoTime() + (long) (seconds * 1e9);
            while (active() && System.nanoTime() < end) {
                show(String.format(Locale.US, "%s  %.0f ticks/s", label, motor.getVelocity()));
            }
        }

        /** 폐루프로 low 속도에서 안정시킨 뒤 target으로 스텝을 주고 응답을 측정한다. low < 0이면 정지 상태에서 시작. */
        StepStats step(double low, double seconds, double windowSec) {
            if (low > 0) {
                motor.setVelocity(low);
                hold(1.5, "준비 (" + Math.round(low) + ")");
            } else {
                coastToStop();
                applyPidfKeep();
            }
            StepStats s = new StepStats();
            s.peak = -Double.MAX_VALUE;
            double sum = 0, sumSq = 0;
            int count = 0;

            motor.setVelocity(cfg.target);
            long start = System.nanoTime(), last = start;
            double filtered = motor.getVelocity();
            while (active()) {
                long now = System.nanoTime();
                double t = (now - start) / 1e9;
                if (t >= seconds) break;
                // 엔코더 속도는 순간 튐이 크다 → 짧은 EMA로 걸러서 판단 (튐 때문에 진동으로 오판하지 않게)
                double dt = (now - last) / 1e9;
                last = now;
                double alpha = dt / (VELOCITY_FILTER_SEC + dt);
                filtered += alpha * (motor.getVelocity() - filtered);

                s.peak = Math.max(s.peak, filtered);
                if (Double.isNaN(s.riseSec) && filtered >= cfg.target - cfg.tolerance) s.riseSec = t;
                if (t >= seconds - windowSec) {
                    sum += filtered;
                    sumSq += filtered * filtered;
                    count++;
                }
                show(String.format(Locale.US, "스텝 응답  %.0f / %.0f ticks/s", filtered, cfg.target));
            }
            if (count > 0) {
                s.windowMean = sum / count;
                s.windowStd = Math.sqrt(Math.max(0, sumSq / count - s.windowMean * s.windowMean));
            }
            return s;
        }

        // coastToStop()이 파워 0으로 바꿔도 PIDF 계수는 유지되므로 모드만 되돌린다.
        private void applyPidfKeep() { motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }

        /** 오버슈트 / 진동이 기준 안에 있는지 (= P나 I가 너무 크지 않은지). 느린 건 여기서 실패로 보지 않는다. */
        boolean stable(StepStats s) {
            return s.overshootPercent(cfg.target) <= cfg.overshootPercent
                    && s.noise() <= cfg.tolerance / 2;
        }

        String describe(StepStats s) {
            return String.format(Locale.US, "rise %s  overshoot %.1f%%  σ %.0f  sse %.0f",
                    Double.isNaN(s.riseSec) ? "미도달" : String.format(Locale.US, "%.2fs", s.riseSec),
                    s.overshootPercent(cfg.target), s.noise(), s.steadyError(cfg.target));
        }

        /** 왜 불안정한지 한 단어로. */
        String failReason(StepStats s) {
            if (s.overshootPercent(cfg.target) > cfg.overshootPercent) return "오버슈트";
            return "진동";
        }
    }

    // ───────────────────────── 1) F ─────────────────────────

    static final class FeedforwardId extends ShooterOpMode<FeedforwardResult> {
        private static final double[] POWERS = {0.3, 0.45, 0.6, 0.75, 0.9, 1.0};

        FeedforwardId(Config cfg) {
            super("Feedforward (F)", "여러 파워로 슈터를 돌려 파워-속도 관계를 측정하고 F를 계산합니다. 약 30초.", cfg);
        }

        @Override
        FeedforwardResult runShooter() {
            List<double[]> samples = new ArrayList<>(); // {power@12V, velocity}
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            for (int k = 0; k < POWERS.length && active(); k++) {
                double power = POWERS[k];
                status = String.format(Locale.US, "F 측정 %d/%d  power %.2f", k + 1, POWERS.length, power);
                motor.setPower(power);
                waitSteady();
                // 0.6초 평균
                double vSum = 0, voltSum = 0;
                int n = 0;
                long end = System.nanoTime() + 600_000_000L;
                while (active() && System.nanoTime() < end) {
                    vSum += motor.getVelocity();
                    voltSum += voltage();
                    n++;
                    show(String.format(Locale.US, "평균 측정  %.0f ticks/s", motor.getVelocity()));
                }
                if (n == 0) break;
                double v = vSum / n, volt = voltSum / n;
                samples.add(new double[]{power * volt / 12.0, v});
            }
            motor.setPower(0);
            if (!active() || samples.size() < 2) return null;

            // 최소제곱 직선: velocity = slope * power12 + intercept
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (double[] s : samples) { sx += s[0]; sy += s[1]; sxx += s[0] * s[0]; sxy += s[0] * s[1]; }
            int n = samples.size();
            FeedforwardResult r = new FeedforwardResult();
            r.slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
            r.intercept = (sy - r.slope * sx) / n;
            r.maxVelocity12V = r.slope + r.intercept;
            r.powerForTarget12V = (cfg.target - r.intercept) / r.slope;
            r.f = 32767.0 * r.powerForTarget12V / cfg.target;
            coastToStop();
            return r;
        }

        /** 속도 변화가 0.3초 동안 1% 이내가 될 때까지(최대 3초) 기다린다. */
        private void waitSteady() {
            long start = System.nanoTime();
            double ref = motor.getVelocity();
            long refTime = start;
            while (active() && System.nanoTime() - start < 3_000_000_000L) {
                double v = motor.getVelocity();
                show(String.format(Locale.US, "안정화 대기  %.0f ticks/s", v));
                if (System.nanoTime() - refTime >= 300_000_000L) {
                    if (System.nanoTime() - start > 800_000_000L && Math.abs(v - ref) <= Math.max(10, Math.abs(ref) * 0.01)) return;
                    ref = v;
                    refTime = System.nanoTime();
                }
            }
        }
    }

    // ───────────────────────── 2) P ─────────────────────────

    static final class PSearch extends ShooterOpMode<Double> {
        /** 이보다 작은 P는 사실상 F만으로 도는 것과 같아 의미가 없다. */
        private static final double MIN_P = 1.0, MAX_P = 256;
        private static final double GROW = 2.0;
        /** 연속으로 이만큼 불안정하면 더 키워도 나빠지기만 하므로 중단. */
        private static final int MAX_UNSTABLE_IN_ROW = 2;
        private final double f;

        PSearch(Config cfg, double f) {
            super("Proportional (P)", "목표의 60%에서 목표로 올리는 스텝을 P = 1, 2, 4, 8 … 로 반복하며 오버슈트/진동 없이 가장 빨리 회복하는 P를 찾습니다. 약 40초.", cfg);
            this.f = f;
        }

        @Override
        Double runShooter() {
            double best = Double.NaN, bestScore = Double.MAX_VALUE;
            int unstableInRow = 0, trial = 0;
            for (double p = MIN_P; p <= MAX_P && active(); p *= GROW) {
                trial++;
                status = String.format(Locale.US, "P 탐색 #%d  P = %s  (best %s)", trial, fmt(p),
                        Double.isNaN(best) ? "-" : fmt(best));
                applyPidf(p, 0, 0, f);
                StepStats s = step(cfg.target * STEP_LOW_RATIO, 2.0, 0.7);
                if (!active()) return null;

                if (stable(s)) {
                    unstableInRow = 0;
                    // 빨리 도달할수록 좋음. 못 도달했으면 남은 오차가 작을수록 좋음(항상 도달한 것보다 뒤).
                    double score = Double.isNaN(s.riseSec)
                            ? 100 + Math.abs(s.steadyError(cfg.target))
                            : s.riseSec;
                    if (score < bestScore) { bestScore = score; best = p; }
                    telemetry.log().add("P " + fmt(p) + " OK    " + describe(s));
                } else {
                    unstableInRow++;
                    telemetry.log().add("P " + fmt(p) + " " + failReason(s) + "  " + describe(s));
                    if (unstableInRow >= MAX_UNSTABLE_IN_ROW) break;
                }
            }
            coastToStop();
            if (!active()) return null;
            if (Double.isNaN(best)) {
                telemetry.log().add("P = " + fmt(MIN_P) + "부터 불안정 → 측정 노이즈가 크거나 Tolerance가 너무 작습니다. MIN_P 사용.");
                return MIN_P;
            }
            return best;
        }
    }

    // ───────────────────────── 3) I ─────────────────────────

    static final class ISearch extends ShooterOpMode<Double> {
        private static final double[] CANDIDATES = {0, 0.05, 0.1, 0.25, 0.5, 1.0};
        private final double p, f;

        ISearch(Config cfg, double p, double f) {
            super("Integral (I)", "정상상태 오차가 남는지 보고, 오차를 없애는 가장 작은 I를 찾습니다. 약 30초.", cfg);
            this.p = p;
            this.f = f;
        }

        @Override
        Double runShooter() {
            double goodError = Math.max(5, cfg.tolerance * 0.25);
            double bestI = 0, bestErr = Double.MAX_VALUE;
            for (int k = 0; k < CANDIDATES.length && active(); k++) {
                double i = CANDIDATES[k];
                status = String.format(Locale.US, "I 탐색 %d/%d  I = %s  (P = %s)", k + 1, CANDIDATES.length, fmt(i), fmt(p));
                applyPidf(p, i, 0, f);
                StepStats s = step(cfg.target * STEP_LOW_RATIO, 3.0, 1.0);
                if (!active()) return null;
                double err = Math.abs(s.steadyError(cfg.target));
                boolean ok = stable(s);
                telemetry.log().add("I " + fmt(i) + (ok ? " OK    " : " " + failReason(s) + "  ") + describe(s));

                if (!ok) break;                        // I가 커서 오버슈트/진동 → 더 키우지 않음
                if (err < bestErr) { bestErr = err; bestI = i; }
                if (err <= goodError) break;           // 충분히 작음 → 가장 작은 I로 확정
            }
            coastToStop();
            return active() ? bestI : null;
        }
    }

    // ───────────────────────── 4) 검증 ─────────────────────────

    static final class Verify extends ShooterOpMode<Verification> {
        private final double p, i, f;

        Verify(Config cfg, double p, double i, double f) {
            super("Verification", "찾은 값으로 정지 → 목표, 60% → 목표 응답을 측정합니다. 약 15초.", cfg);
            this.p = p;
            this.i = i;
            this.f = f;
        }

        @Override
        Verification runShooter() {
            applyPidf(p, i, 0, f);
            status = "검증 1/2: 정지 → 목표";
            StepStats fromZero = step(-1, 3.0, 1.0);
            status = "검증 2/2: 60% → 목표";
            StepStats recovery = step(cfg.target * STEP_LOW_RATIO, 3.0, 1.0);
            coastToStop();
            if (!active()) return null;

            Verification v = new Verification();
            v.spinUpSec = fromZero.riseSec;
            v.recoverySec = recovery.riseSec;
            v.overshootPercent = Math.max(fromZero.overshootPercent(cfg.target), recovery.overshootPercent(cfg.target));
            v.steadyError = recovery.steadyError(cfg.target);
            v.noise = Math.max(fromZero.noise(), recovery.noise());
            return v;
        }
    }
}
