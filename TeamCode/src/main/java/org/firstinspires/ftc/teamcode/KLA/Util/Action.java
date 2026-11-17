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

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.infinite;

/**
 * NextFTC mechanism containing the intake, shooter, and game-piece servos.
 *
 * <p>인테이크 모터 출력은 {@link #update()} 한 곳에서만 결정한다. 버튼 커맨드는 "눌려 있다"는 상태만
 * 바꾸고 모터에 직접 쓰지 않으므로, 발사 커맨드와 인테이크 커맨드가 서로 막거나 덮어쓰지 않는다.
 *
 * <pre>
 *   발사 중이 아닐 때 : A = 인테이크 정방향, B = 역방향
 *   발사 중          : 슈터가 목표 속도에 도달하거나, SPINUP_TIMEOUT_S 가 지나거나,
 *                      A(인테이크)를 누르면 → 블로커 열고 인테이크 풀파워로 급탄 (트리거 놓을 때까지 유지)
 * </pre>
 */
@Configurable
public class Action implements Mechanism {
    private static final long PIDF_VOLTAGE_REFRESH_NS = 500_000_000L;
    private static final double PIDF_F_UPDATE_THRESHOLD = 0.01;

    public static double P = 20, I = 0.05, D = 0.0, F = 15.5;
    public static double SHOOTING_VELOCITY = 1500;
    public static double SHOOTER_VELOCITY_TOLERANCE = 50;
    /** 발사 시작 후 이 시간(초)이 지나면 속도 미달이어도 급탄한다. 0 이하면 끔(속도 도달 또는 A 버튼으로만 급탄). */
    public static double SPINUP_TIMEOUT_S = 1.5;
    public static double Shoot_Intake_Power = 1.0;
    public static double flower_init = 0.5, flower_open = 0.98;
    public static double blocker_open = 0.4, blocker_close = 0.25;
    public static double Intake_Power = 0.7, IntakeR_Power = -0.3;

    private final HardwareMap hardwareMap;
    private final DcMotorEx shooter;
    private final DcMotor intake;
    private final Servo blocker, flower;
    private long nextPidfVoltageRefreshNs;
    private double cachedBatteryVoltage;
    private double appliedP = Double.NaN, appliedI = Double.NaN;
    private double appliedD = Double.NaN, appliedF = Double.NaN;

    // 버튼/발사 상태. 실제 출력은 update()가 이 값들로 결정한다.
    private boolean intakeForwardHeld, intakeReverseHeld;
    private boolean shooting, feeding;
    /** 발사 전에 슈터만 미리 돌리는 상태(급탄은 하지 않음). 오토에서 이동 중 예열용. */
    private boolean shooterArmed;
    private long shootStartNs;
    private String feedReason = "-";

    // 같은 값을 매 루프 다시 보내지 않도록 마지막으로 쓴 값을 기억한다.
    private double lastIntakePower = Double.NaN;

    public Action(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        flower = hardwareMap.get(Servo.class, "flower");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        refreshShooterPidf(true);
        shooter.setVelocity(0);
        setIntakePower(0);
        flower.setPosition(flower_init);
        blocker.setPosition(blocker_close);
    }

    @Override public void periodic() {
        refreshShooterPidf(false);
        update();
    }

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

    /**
     * 현재 상태로 슈터/블로커/인테이크 출력을 정한다. 매 루프 periodic()에서 호출되고,
     * 커맨드 실행 중에도 호출되어 버튼 반응이 한 루프 늦지 않게 한다. 여러 번 불려도 결과는 같다.
     */
    public void update() {
        if (shooting) {
            shooter.setVelocity(SHOOTING_VELOCITY);
            if (!feeding) {
                double elapsedS = (System.nanoTime() - shootStartNs) / 1e9;
                if (shooterAtSpeed()) startFeeding("속도 도달");
                else if (intakeForwardHeld) startFeeding("A 강제");
                else if (SPINUP_TIMEOUT_S > 0 && elapsedS >= SPINUP_TIMEOUT_S) startFeeding("타임아웃");
            }
            if (feeding) setIntakePower(Shoot_Intake_Power);
            else if (intakeReverseHeld) setIntakePower(IntakeR_Power);
            else setIntakePower(0);
        } else {
            if (shooterArmed) shooter.setVelocity(SHOOTING_VELOCITY);
            if (intakeForwardHeld) setIntakePower(Intake_Power);
            else if (intakeReverseHeld) setIntakePower(IntakeR_Power);
            else setIntakePower(0);
        }
    }

    private void startFeeding(String reason) {
        feeding = true;
        feedReason = reason;
        blocker.setPosition(blocker_open);
    }

    private void setIntakePower(double power) {
        if (Double.compare(power, lastIntakePower) != 0) {
            intake.setPower(power);
            lastIntakePower = power;
        }
    }

    /** 누르고 있는 동안 인테이크 정방향. 발사 중에 누르면 속도와 상관없이 즉시 급탄한다. */
    public CommandBuilder intakeForward() {
        return infinite(this::update)
                .setStart(() -> { intakeForwardHeld = true; update(); })
                .setEnd(endCondition -> { intakeForwardHeld = false; update(); });
    }

    /** 누르고 있는 동안 인테이크 역방향. 급탄 중에는 무시된다. */
    public CommandBuilder intakeReverse() {
        return infinite(this::update)
                .setStart(() -> { intakeReverseHeld = true; update(); })
                .setEnd(endCondition -> { intakeReverseHeld = false; update(); });
    }

    /** 버튼 상태를 모두 해제한다(안전망). 발사 중 급탄은 멈추지 않는다. */
    public CommandBuilder intakeOff() {
        return instant(() -> { intakeForwardHeld = false; intakeReverseHeld = false; update(); });
    }

    // ── 오토용: 버튼 없이 상태만 켜고 끄는 instant 커맨드 ──
    /** 인테이크 정방향 켜기. intakeStop()까지 유지된다. */
    public CommandBuilder intakeOn() {
        return instant(() -> { intakeReverseHeld = false; intakeForwardHeld = true; update(); });
    }
    /** 인테이크 끄기. intakeOff()와 같다. */
    public CommandBuilder intakeStop() { return intakeOff(); }

    /** 슈터만 미리 목표 속도로 돌린다(급탄 X). 이후 shoot()이 시작되면 바로 급탄되고, 발사가 끝나면 꺼진다. */
    public CommandBuilder spinUp() {
        return instant(() -> { shooterArmed = true; update(); });
    }

    /** 슈터가 목표 속도에 도달했는지 여부. */
    public boolean shooterAtSpeed() {
        return shooter.getVelocity() >= SHOOTING_VELOCITY - SHOOTER_VELOCITY_TOLERANCE;
    }

    /**
     * 누르고 있는 동안 실행: 슈터를 목표 속도로 돌리고, 급탄 조건(속도 도달 / A 버튼 / 타임아웃)이
     * 되면 블로커를 열고 인테이크를 풀파워로 돌린다. 한 번 급탄을 시작하면 속도가 잠깐 떨어져도
     * 유지하고, 커맨드가 끝날 때(트리거 해제) 슈터를 멈추고 블로커를 닫는다.
     * 인테이크는 requirement로 잡지 않으므로 발사 중에도 인테이크 버튼이 동작한다.
     */
    public CommandBuilder shoot() {
        return infinite(this::update)
                .setStart(() -> {
                    shooting = true;
                    feeding = false;
                    feedReason = "-";
                    shootStartNs = System.nanoTime();
                    update();
                })
                .setEnd(endCondition -> stopShooting())
                .requiring(shooter);
    }

    /** 트리거를 놓았을 때의 안전망. shoot()의 end와 같은 동작. */
    public CommandBuilder stopShooter() { return instant(this::stopShooting); }

    private void stopShooting() {
        if (!shooting && !feeding && !shooterArmed) return;
        shooting = false;
        feeding = false;
        shooterArmed = false;
        shooter.setVelocity(0);
        blocker.setPosition(blocker_close);
        update(); // 인테이크는 버튼 상태로 복귀 (A를 누르고 있으면 계속 돈다)
    }

    public CommandBuilder blockerOpen() { return instant(() -> blocker.setPosition(blocker_open)); }
    public CommandBuilder blockerClose() { return instant(() -> blocker.setPosition(blocker_close)); }
    public CommandBuilder flowerOpen() { return instant(() -> flower.setPosition(flower_open)); }
    public CommandBuilder flowerClose() { return instant(() -> flower.setPosition(flower_init)); }

    // ── 텔레메트리용 ──
    public double shooterVelocity() { return shooter.getVelocity(); }
    public boolean isShooting() { return shooting; }
    public boolean isFeeding() { return feeding; }
    public String feedReason() { return feedReason; }
}
