package org.firstinspires.ftc.teamcode.KLA.Util;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.robot.Mechanism;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.infinite;

/** NextFTC mechanism containing the intake, shooter, and game-piece servos. */
@Configurable
public class Action implements Mechanism {
    private static final long PIDF_VOLTAGE_REFRESH_NS = 500_000_000L;
    private static final double PIDF_F_UPDATE_THRESHOLD = 0.01;

    public static double P = 20, I = 0.05, D = 0.0, F = 15.5;
    public static double SHOOTING_VELOCITY = 1500;
    public static double SHOOTER_VELOCITY_TOLERANCE = 40;
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
        flower.setPosition(flower_init);
        blocker.setPosition(blocker_close);
    }

    @Override public void periodic() { refreshShooterPidf(false); }

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

    /** 시작할 때 블로커를 한 번만 닫는다. 매 루프 닫지 않으므로 인테이크 중에도 dpad 수동 조작이 가능하다. */
    public CommandBuilder intakeForward() {
        return infinite(() -> intake.setPower(Intake_Power))
                .setStart(() -> blocker.setPosition(blocker_close))
                .requiring(intake);
    }
    public CommandBuilder intakeReverse() { return infinite(() -> intake.setPower(IntakeR_Power)).requiring(intake); }
    public CommandBuilder intakeOff() { return instant(() -> intake.setPower(0)).requiring(intake); }

    /** 슈터가 목표 속도에 도달했는지 여부. */
    public boolean shooterAtSpeed() {
        return shooter.getVelocity() >= SHOOTING_VELOCITY - SHOOTER_VELOCITY_TOLERANCE;
    }

    /**
     * 누르고 있는 동안 실행: 슈터를 목표 속도로 돌리고, 도달하면 블로커를 열고 인테이크를 풀 속도로 돌린다.
     * 한 번 열리면 발사 중 속도가 잠깐 떨어져도 닫지 않고, 커맨드가 끝날 때(트리거 해제) 모두 정지한다.
     * 인테이크 커맨드보다 우선순위가 높아서 발사 중에는 인테이크 버튼이 무시된다.
     */
    public CommandBuilder shoot() {
        return infinite(() -> {
                    shooter.setVelocity(SHOOTING_VELOCITY);
                    if (shooterAtSpeed()) {
                        blocker.setPosition(blocker_open);
                        intake.setPower(Shoot_Intake_Power);
                    }
                })
                .setEnd(endCondition -> stopShooting())
                .requiring(shooter, intake)
                .setPriority(1);
    }

    /** 트리거를 놓았을 때의 안전망. shoot()의 end와 같은 동작. */
    public CommandBuilder stopShooter() { return instant(this::stopShooting); }

    private void stopShooting() {
        shooter.setVelocity(0);
        blocker.setPosition(blocker_close);
        intake.setPower(0);
    }

    public CommandBuilder blockerOpen() { return instant(() -> blocker.setPosition(blocker_open)); }
    public CommandBuilder blockerClose() { return instant(() -> blocker.setPosition(blocker_close)); }
    public CommandBuilder flowerOpen() { return instant(() -> flower.setPosition(flower_open)); }
    public CommandBuilder flowerClose() { return instant(() -> flower.setPosition(flower_init)); }
}
