package org.firstinspires.ftc.teamcode.KLA.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class Action {
    private static final long PIDF_VOLTAGE_REFRESH_NS = 500_000_000L;
    private static final double PIDF_F_UPDATE_THRESHOLD = 0.01;

    public static double P = 20;
    public static double I = 0.05;
    public static double D = 0.0;
    public static double F = 15.5;
    public static double SHOOTING_VELOCITY = 1500;
    //// flower
    ////     // 0.98 intake
    ///     // 0.5 init
    ///
    /// blocker
    ///     // 0.4 open
    ///     // 0.25 close
    ///
    public static double flower_init = 0.5;
    public static double flower_open = 0.98;
    public static double blocker_open = 0.4;
    public static double blocker_close = 0.25;
    public static double Intake_Power = 0.4;


    private final DcMotorEx shooter;
    private final DcMotor turret, intake;
    private final HardwareMap hardwareMap;
    Servo blocker,flower,hood;
    Telemetry telemetry;
    private final List<LynxModule> hubs;
    private long nextPidfVoltageRefreshNs;
    private double cachedBatteryVoltage;
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    public Action(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        this.turret = hardwareMap.get(DcMotor.class,"turret");
        this.intake = hardwareMap.get(DcMotor.class,"intake");

        this.blocker = hardwareMap.get(Servo.class,"blocker");
        this.flower = hardwareMap.get(Servo.class,"flower");
        this.hood = hardwareMap.get(Servo.class,"hood");

        this.telemetry = telemetry;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updateShooterPidf(true);
        shooter.setVelocity(0);

        flower.setPosition(flower_init);
        blocker.setPosition(blocker_close);
    }

    /** Refresh battery compensation while the OpMode is running. */
    public void updateShooterPidf(){
        updateShooterPidf(false);
    }

    private void updateShooterPidf(boolean force){
        long now = System.nanoTime();
        if (force || now >= nextPidfVoltageRefreshNs) {
            cachedBatteryVoltage = getBatteryVoltage();
            nextPidfVoltageRefreshNs = now + PIDF_VOLTAGE_REFRESH_NS;
        }

        double compensatedF = cachedBatteryVoltage > 0
                ? F * (12.0 / cachedBatteryVoltage)
                : F;
        boolean gainsChanged = Double.compare(P, appliedP) != 0
                || Double.compare(I, appliedI) != 0
                || Double.compare(D, appliedD) != 0;
        boolean fChanged = Double.isNaN(appliedF)
                || Math.abs(compensatedF - appliedF) >= PIDF_F_UPDATE_THRESHOLD;
        if (force || gainsChanged || fChanged) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, compensatedF);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            appliedP = P;
            appliedI = I;
            appliedD = D;
            appliedF = compensatedF;
        }
    }

    private double getBatteryVoltage(){
        double lowestVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor voltageSensor : hardwareMap.voltageSensor) {
            double voltage = voltageSensor.getVoltage();
            if (voltage > 0 && voltage < lowestVoltage) {
                lowestVoltage = voltage;
            }
        }
        return Double.isInfinite(lowestVoltage) ? 0 : lowestVoltage;
    }

    public void clearBulkCache(){
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void setIntakePower(double power){
        intake.setPower(power);
    }

    public void startShooter(){shooter.setVelocity(SHOOTING_VELOCITY);}

    public void stopShooter(){
        shooter.setVelocity(0);
    }

    // real command

    public void IntakeOn(){
        intake.setPower(Intake_Power);
    }

    public void IntakeOff(){
        intake.setPower(0);
    }

    public void BlockerOpen(){
        blocker.setPosition(blocker_open);
    }

    public void BlockerClose(){
        blocker.setPosition(blocker_close);
    }

    public void FlowerOpen(){
        flower.setPosition(flower_open);
    }

}
