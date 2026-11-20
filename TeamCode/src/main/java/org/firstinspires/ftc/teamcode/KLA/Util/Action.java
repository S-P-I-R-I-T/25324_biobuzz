package org.firstinspires.ftc.teamcode.KLA.Util;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.robot.Mechanism;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.function.DoubleSupplier;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.infinite;

/**
 * NextFTC mechanism containing the intake, shooter, and game-piece servos.
 *
 * <pre>
 *   발사 버튼 누르는 동안 : 슈터 목표 속도 + 블로커 열림, 목표 속도 도달하면 인테이크 자동 급탄 (Intake_Power)
 *   발사 버튼 떼면        : 슈터 예열/정지 + 블로커 닫힘 + 인테이크 정지
 *   A = 정방향, B = 역방향 (언제든 수동으로 덮어씀)
 * </pre>
 */
@Configurable
public class Action implements Mechanism {
    private static final long PIDF_VOLTAGE_REFRESH_NS = 500_000_000L;
    private static final double PIDF_F_UPDATE_THRESHOLD = 0.01;

    public static double P = 100, I = 0, D = 0.0, F = 18.5;
    /** true: 거리 → ShotTable로 후드 자동. false: hood_pos 수동 (표 값 찾을 때). 속도는 항상 SHOOTING_VELOCITY. */
    public static boolean AUTO_SHOT = true;
    public static double SHOOTING_VELOCITY = 1300;
    /** TeleOp 예열 속도 (발사 안 할 때, ticks/s). */
    public static double IDLE_VELOCITY = 800;
    public static double SHOOTER_VELOCITY_TOLERANCE = 50;
    public static double Shoot_Intake_Power = 0.7;
    public static double flower_init = 0.465, flower_open = 0.945, flower_Nachi = 0.7;
    public static double blocker_open = 0.99, blocker_close = 0.91;
    public static double Intake_Power = 0.7, IntakeR_Power = -0.4;
    /** 수동 모드(AUTO_SHOT = false) 후드 위치. */
    public static double hood_pos = 0.84;
    public static double HOOD_MIN = 0.80, HOOD_MAX = 0.97;
    public static double HOOD_FINE_STEP = 0.01, HOOD_COARSE_STEP = 0.05;
    /** 연발 중 슈터 속도가 떨어진 만큼 후드를 올림: 1 tick/s 떨어질 때마다 +이 값. 0이면 보정 끔. */
    public static double HOOD_COMP_PER_TPS = 0.0004;
    public static double HOOD_COMP_DEADBAND_TPS = 20;
    public static double HOOD_COMP_FILTER_SEC = 0.05;

    private final HardwareMap hardwareMap;
    private final DcMotorEx shooter;
    private final DcMotor intake;
    private final Servo blocker, flower, hood;
    private double appliedHoodPos = Double.NaN, hoodCommand;
    private boolean shooting, keepSpinning, feeding;
    private double lastIdleVelocity = Double.NaN;
    private DoubleSupplier distanceSupplier;
    private double distance = Double.NaN, targetVelocity = SHOOTING_VELOCITY, baseHood = hood_pos;
    private boolean autoActive;
    private double filteredVelocity;
    private long lastHoodNs = System.nanoTime();
    private long nextPidfVoltageRefreshNs;
    private double cachedBatteryVoltage;
    private double appliedP = Double.NaN, appliedI = Double.NaN;
    private double appliedD = Double.NaN, appliedF = Double.NaN;

    public Action(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        flower = hardwareMap.get(Servo.class, "flower");
        hood = hardwareMap.get(Servo.class, "hood");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        refreshShooterPidf(true);
        shooter.setVelocity(0);
        intake.setPower(0);
        flower.setPosition(flower_init);
        blocker.setPosition(blocker_close);
        applyHood();
    }

    @Override public void periodic() {
        refreshShooterPidf(false);
        updateShotParams();
        if (keepSpinning && !shooting && IDLE_VELOCITY != lastIdleVelocity) {
            shooter.setVelocity(IDLE_VELOCITY);
            lastIdleVelocity = IDLE_VELOCITY;
        }
        applyHood();
    }

    /** 목표까지 거리(inch)를 주는 함수. 없으면 수동 값 사용. */
    public void setDistanceSupplier(DoubleSupplier supplier) { distanceSupplier = supplier; }

    private void updateShotParams() {
        distance = distanceSupplier != null ? distanceSupplier.getAsDouble() : Double.NaN;
        autoActive = AUTO_SHOT && !Double.isNaN(distance) && ShotTable.valid();
        targetVelocity = SHOOTING_VELOCITY;
        baseHood = clampHood(autoActive ? ShotTable.hood(distance) : hood_pos);
    }

    // ── 후드 ──
    private void applyHood() {
        hood_pos = clampHood(hood_pos);

        long now = System.nanoTime();
        double dt = (now - lastHoodNs) / 1e9;
        lastHoodNs = now;
        filteredVelocity += dt / (HOOD_COMP_FILTER_SEC + dt) * (shooter.getVelocity() - filteredVelocity);

        double comp = 0;
        if (shooting) {
            double drop = targetVelocity - filteredVelocity - HOOD_COMP_DEADBAND_TPS;
            if (drop > 0) comp = drop * HOOD_COMP_PER_TPS;
        }
        hoodCommand = clampHood(baseHood + comp);
        if (Double.isNaN(appliedHoodPos) || Math.abs(hoodCommand - appliedHoodPos) >= 0.002) {
            hood.setPosition(hoodCommand);
            appliedHoodPos = hoodCommand;
        }
    }

    private static double clampHood(double pos) { return Math.max(HOOD_MIN, Math.min(HOOD_MAX, pos)); }

    /** 후드를 delta만큼 움직인다(0~1로 제한). */
    public CommandBuilder hoodAdjust(double delta) {
        return instant(() -> { hood_pos += delta; applyHood(); });
    }

    /** 후드를 지정 위치로 (나중에 거리별 값이 정해지면 사용). */
    public CommandBuilder hoodTo(double position) {
        return instant(() -> { hood_pos = position; applyHood(); });
    }

    public double hoodPosition() { return hood_pos; }
    public double hoodCommand() { return hoodCommand; }

    private void refreshShooterPidf(boolean force) {
        long now = System.nanoTime();
        if (force || now >= nextPidfVoltageRefreshNs) {
            double lowest = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (voltage > 0 && voltage < lowest) lowest = voltage;
            }
            cachedBatteryVoltage = Double.isInfinite(lowest) ? 0 : lowest;
            nextPidfVoltageRefreshNs = now + PIDF_VOLTAGE_REFRESH_NS;
        }
        double compensatedF = cachedBatteryVoltage > 0 ? F * (12.0 / cachedBatteryVoltage) : F;
        boolean changed = Double.compare(P, appliedP) != 0 || Double.compare(I, appliedI) != 0
                || Double.compare(D, appliedD) != 0 || Double.isNaN(appliedF)
                || Math.abs(compensatedF - appliedF) >= PIDF_F_UPDATE_THRESHOLD;
        if (force || changed) {
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, I, D, compensatedF));
            appliedP = P; appliedI = I; appliedD = D; appliedF = compensatedF;
        }
    }

    // ── 인테이크 (TeleOp: 누르는 동안) ──
    public CommandBuilder intakeForward() {
        return infinite(() -> intake.setPower(Intake_Power))
                .setEnd(endCondition -> intake.setPower(0))
                .requiring(intake);
    }
    public CommandBuilder intakeReverse() {
        return infinite(() -> intake.setPower(IntakeR_Power))
                .setEnd(endCondition -> intake.setPower(0))
                .requiring(intake);
    }

    // ── 인테이크 (오토: 켜고 끄기) ──
    public CommandBuilder intakeOn() { return instant(() -> intake.setPower(Intake_Power)); }
    /** 발사할 때 급탄 (TeleOp 인테이크와 같은 0.7). */
    public CommandBuilder feedOn() { return instant(() -> intake.setPower(Shoot_Intake_Power)); }
    public CommandBuilder intakeStop() { return instant(() -> intake.setPower(0)); }

    // ── 슈터 ──
    /** 슈터가 목표 속도에 도달했는지 여부 (텔레메트리용). */
    public boolean shooterAtSpeed() {
        return shooter.getVelocity() >= targetVelocity - SHOOTER_VELOCITY_TOLERANCE;
    }

    /** 누르고 있는 동안: 슈터 목표 속도 + 블로커 열림, 목표 속도 도달하면 그때부터 인테이크 급탄(계속). 끝나면 전부 정지. */
    public CommandBuilder shoot() {
        return infinite(() -> {
                    shooter.setVelocity(targetVelocity);
                    if (!feeding && shooterAtSpeed()) {
                        feeding = true;
                        intake.setPower(Intake_Power);
                    }
                })
                .setStart(() -> { shooting = true; feeding = false; blocker.setPosition(blocker_open); })
                .setEnd(endCondition -> {
                    if (feeding) intake.setPower(0);
                    feeding = false;
                    stopShooting();
                })
                .requiring(shooter);
    }

    /** 슈터만 미리 돌린다(블로커는 닫힌 채). 오토에서 이동 중 예열용. shoot()이 끝나면 함께 꺼진다. */
    /** true면 발사 안 할 때도 슈터를 IDLE_VELOCITY로 계속 돌림 (TeleOp 예열). 블로커는 닫힌 채. */
    public void setKeepSpinning(boolean on) {
        keepSpinning = on;
        lastIdleVelocity = Double.NaN;
        if (!on && !shooting) shooter.setVelocity(0);
    }

    public CommandBuilder spinUp() { return instant(() -> { updateShotParams(); shooter.setVelocity(targetVelocity); }); }

    /** 버튼을 놓았을 때의 안전망. shoot()의 end와 같은 동작. */
    public CommandBuilder stopShooter() { return instant(this::stopShooting); }

    private void stopShooting() {
        shooting = false;
        lastIdleVelocity = keepSpinning ? IDLE_VELOCITY : Double.NaN;
        shooter.setVelocity(keepSpinning ? IDLE_VELOCITY : 0);
        blocker.setPosition(blocker_close);
    }

    public CommandBuilder blockerOpen() { return instant(() -> blocker.setPosition(blocker_open)); }
    public CommandBuilder blockerClose() { return instant(() -> blocker.setPosition(blocker_close)); }
    public CommandBuilder flowerOpen() { return instant(() -> flower.setPosition(flower_open)); }
    public CommandBuilder flowerClose() { return instant(() -> flower.setPosition(flower_Nachi)); }

    public double shooterVelocity() { return shooter.getVelocity(); }
    /** 지금 돌려야 할 속도(발사 중이면 목표, 아니면 예열)에 도달했는지. */
    public boolean shooterSpunUp() {
        double want = shooting ? targetVelocity : (keepSpinning ? IDLE_VELOCITY : targetVelocity);
        return shooter.getVelocity() >= want - SHOOTER_VELOCITY_TOLERANCE;
    }
    public double targetVelocity() { return targetVelocity; }
    public double baseHood() { return baseHood; }
    public double distance() { return distance; }
    public boolean autoShotActive() { return autoActive; }
}
