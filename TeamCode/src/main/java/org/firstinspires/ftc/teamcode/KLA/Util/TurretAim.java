package org.firstinspires.ftc.teamcode.KLA.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.robot.Mechanism;

/**
 * 터렛 PID (오차 단위: °, 출력: 모터 파워).
 * localizer가 있으면 진영 셀 자동 조준, localizer가 null이거나 setManualTarget()이면 그 각도 유지.
 * 0° = resetEncoder() 할 때의 방향(로봇 정면), + = 반시계.
 * 튜닝 순서: kS → kP → kD → (필요하면) kI. TurretTuner 참고.
 */
@Configurable
public class TurretAim implements Mechanism {
    public static boolean MOTOR_REVERSED = false;
    public static double TICKS_PER_REV = 537.7, GEAR_RATIO = 135.0 / 89;
    public static double MAX_ANGLE_DEG = 200;

    public static double kP = 0.03, kI = 0.0, kD = 0.0007, kS = 0.07;
    public static double MAX_POWER = 0.8;
    public static double TOLERANCE_DEG = 0.5;
    /** 오차가 이 안일 때만 I 누적 (멀리 있을 때 와인드업 방지). */
    public static double I_ZONE_DEG = 5;

    private final DcMotorEx motor;
    private final Localizer localizer;
    private final Alliance alliance;

    private Pose pose, target;
    private Double manualTargetDeg = null;
    private double targetDeg, currentDeg, power, integral;
    private int currentTicks;
    private long lastNs = System.nanoTime();

    /** init 때 호출: 지금 터렛 방향을 0°로. */
    public static void resetEncoder(HardwareMap hardwareMap) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, "turret");
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public TurretAim(HardwareMap hardwareMap, Localizer localizer, Alliance alliance) {
        this.localizer = localizer;
        this.alliance = alliance;
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (localizer == null) manualTargetDeg = 0.0;
    }

    @Override public void periodic() {
        currentTicks = motor.getCurrentPosition();
        currentDeg = currentTicks / ticksPerDegree();
        if (localizer != null) {
            pose = localizer.pose();
            target = alliance.targetFor(pose);
        }

        if (manualTargetDeg != null || pose == null) {
            targetDeg = clamp(manualTargetDeg != null ? manualTargetDeg : 0);
        } else {
            double aim = Math.toDegrees(Math.atan2(target.y() - pose.y(), target.x() - pose.x()) - pose.heading());
            targetDeg = closestDeg(aim, currentDeg);
        }

        long now = System.nanoTime();
        double dt = Math.min((now - lastNs) / 1e9, 0.1);
        lastNs = now;
        power = pid(targetDeg - currentDeg, motor.getVelocity() / ticksPerDegree(), dt);
        motor.setPower(power);
    }

    private double pid(double error, double velocityDegPerSec, double dt) {
        if (Math.abs(error) <= TOLERANCE_DEG) {
            integral = 0;
            return 0;
        }
        if (Math.abs(error) < I_ZONE_DEG) integral += error * dt;
        else integral = 0;

        // D는 측정 속도 기준 → 목표가 갑자기 바뀌어도 튀지 않음
        double out = kP * error + kI * integral - kD * velocityDegPerSec + kS * Math.signum(error);
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, out));
    }

    /** ±360° 후보 중 리밋 안에서 현재 각도와 가장 가까운 것 (뒤로 지날 때 한 바퀴 도는 것 방지). */
    private static double closestDeg(double aimDeg, double current) {
        aimDeg = Math.IEEEremainder(aimDeg, 360);
        Double best = null;
        for (int k = -1; k <= 1; k++) {
            double c = aimDeg + k * 360;
            if (Math.abs(c) <= MAX_ANGLE_DEG && (best == null || Math.abs(c - current) < Math.abs(best - current))) best = c;
        }
        return best != null ? best : clamp(aimDeg);
    }

    private static double clamp(double deg) { return Math.max(-MAX_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, deg)); }
    private static double ticksPerDegree() { return TICKS_PER_REV * GEAR_RATIO / 360.0; }

    /** null이 아니면 이 각도(°) 유지. null이면 필드 조준 (localizer 있을 때). */
    public void setManualTarget(Double deg) {
        manualTargetDeg = deg;
        integral = 0;
    }

    public void stop() {
        power = 0;
        motor.setPower(0);
    }

    public Pose pose() { return pose; }
    public String targetName() {
        if (manualTargetDeg != null || target == null) return "Manual";
        return target == alliance.garden() ? "Garden" : "Flower";
    }
    public double targetDeg() { return targetDeg; }
    public double currentDeg() { return currentDeg; }
    public double power() { return power; }
    /** 필드 조준 중이고 오차가 maxErrorDeg 이내. */
    public boolean aimed(double maxErrorDeg) {
        return manualTargetDeg == null && pose != null && Math.abs(targetDeg - currentDeg) <= maxErrorDeg;
    }
    /** 엔코더 원래 값. */
    public int currentTicks() { return currentTicks; }
    public int targetTicks() { return (int) Math.round(targetDeg * ticksPerDegree()); }
}
