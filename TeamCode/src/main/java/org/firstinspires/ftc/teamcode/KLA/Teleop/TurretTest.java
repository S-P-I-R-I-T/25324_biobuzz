package org.firstinspires.ftc.teamcode.KLA.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.KLA.Util.Drive;
import org.firstinspires.ftc.teamcode.KLA.Util.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Turret 클래스 테스트.
 *
 * <pre>
 *   gamepad1     주행 (Drive.controlWithPad 그대로)
 *
 *   gamepad2
 *   A            자동 조준 ON / OFF
 *   D-Pad 좌/우   조준할 목표 변경
 *   왼쪽 스틱 X   (자동 조준 OFF일 때) 터렛 수동 회전
 *   Y            터렛 엔코더 0점 재설정 (지금 방향 = 로봇 정면)
 *   B            로봇 위치를 시작 포즈로 재설정
 * </pre>
 *
 * 전제: OpMode를 시작할 때 터렛이 로봇 정면을 보고 있고, 로봇이 START 포즈에 놓여 있다.
 *
 * 처음 확인할 것 (자동 조준 OFF 상태에서):
 *   1. 터렛을 손이나 스틱으로 왼쪽(위에서 봤을 때 반시계)으로 돌리면 "터렛 각도"가 +로 늘어야 한다.
 *      반대면 MOTOR_REVERSED 를 바꾼다.
 *   2. 터렛을 정확히 90° 돌렸을 때 "터렛 각도"가 90 근처여야 한다. 아니면 Turret의 TPR / 기어비가 틀린 것.
 */
@Configurable
@TeleOp(name = "Turret Test", group = "Tuning")
public class TurretTest extends OpMode {
    public static String MOTOR_NAME = "turret";
    public static boolean MOTOR_REVERSED = false;
    public static double AIM_POWER = 0.6;
    public static double MANUAL_POWER = 0.3;
    /** 배선 보호용 소프트 리밋. 정면 기준 ±이 각도를 넘는 목표는 리밋에서 멈춘다. */
    public static double MAX_ANGLE_DEG = 180;

    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 0;

    private final Turret turret = new Turret();
    private final String[] targetNames = {"RED Garden", "RED Flower", "BLUE Garden", "BLUE Flower"};
    private final Pose[] targets = {
            new Pose(turret.RED_Garden_Cell_X, turret.RED_Garden_Cell_Y),
            new Pose(turret.RED_Flower_Cell_X, turret.RED_Flower_Cell_Y),
            new Pose(turret.BLUE_Garden_Cell_X, turret.BLUE_Garden_Cell_Y),
            new Pose(turret.BLUE_Flower_Cell_X, turret.BLUE_Flower_Cell_Y),
    };

    private DcMotorEx motor;
    private Localizer localizer;
    private Drive drive;

    private int targetIndex = 0;
    private boolean autoAim = false;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetTurretZero();

        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(startPose());

        drive = new Drive(hardwareMap, telemetry, gamepad1);
        drive.init();

        telemetry.addLine("터렛을 로봇 정면으로 맞추고, 로봇을 시작 포즈에 놓은 뒤 시작하세요");
    }

    @Override
    public void loop() {
        drive.clearBulkCache();
        localizer.update();
        drive.controlWithPad();
        handleInput();

        Pose botPose = localizer.pose();
        Pose target = targets[targetIndex];

        // ── Turret 사용법의 핵심: 현재 포즈와 목표 좌표를 넣으면 터렛 모터의 목표 틱이 나온다 ──
        int limitTicks = degreesToTicks(MAX_ANGLE_DEG);
        int aimTicks = turret.alignTarget(botPose, target);
        int targetTicks = Math.max(-limitTicks, Math.min(limitTicks, aimTicks));

        if (autoAim) {
            motor.setTargetPosition(targetTicks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(AIM_POWER);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(manualPower(limitTicks));
        }

        showTelemetry(botPose, target, aimTicks, targetTicks);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    private void handleInput() {
        if (gamepad2.aWasPressed()) autoAim = !autoAim;
        if (gamepad2.dpadRightWasPressed()) targetIndex = (targetIndex + 1) % targets.length;
        if (gamepad2.dpadLeftWasPressed()) targetIndex = (targetIndex + targets.length - 1) % targets.length;
        if (gamepad2.bWasPressed()) localizer.setPose(startPose());
        if (gamepad2.yWasPressed()) {
            autoAim = false;
            resetTurretZero();
        }
    }

    /** 스틱 왼쪽 = 반시계(+). 소프트 리밋 바깥 방향으로는 움직이지 않는다. */
    private double manualPower(int limitTicks) {
        double power = -gamepad2.left_stick_x * MANUAL_POWER;
        int position = motor.getCurrentPosition();
        if (position >= limitTicks && power > 0) return 0;
        if (position <= -limitTicks && power < 0) return 0;
        return power;
    }

    private void resetTurretZero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private double ticksPerDegree() {
        return turret.SHOOTER_ANGLE_TPR * turret.GEAR_RATIO / 360.0;
    }

    private int degreesToTicks(double degrees) {
        return (int) (degrees * ticksPerDegree());
    }

    private void showTelemetry(Pose botPose, Pose target, int aimTicks, int targetTicks) {
        int currentTicks = motor.getCurrentPosition();

        telemetry.addData("자동 조준", "%s  (A)", autoAim ? "ON" : "OFF — 왼쪽 스틱으로 수동 회전");
        telemetry.addData("목표", "%s  (%.1f, %.1f)  (D-Pad 좌/우)",
                targetNames[targetIndex], target.x(), target.y());

        telemetry.addLine();
        telemetry.addData("로봇 포즈", "x %.1f  y %.1f  heading %.1f°",
                botPose.x(), botPose.y(), Math.toDegrees(botPose.heading()));

        telemetry.addLine();
        telemetry.addLine("── 터렛 (정면 = 0°, 반시계 = +) ──");
        telemetry.addData("목표 각도", "%.1f°  (%d ticks)%s", targetTicks / ticksPerDegree(), targetTicks,
                aimTicks != targetTicks ? "  [리밋에 걸림]" : "");
        telemetry.addData("터렛 각도", "%.1f°  (%d ticks)", currentTicks / ticksPerDegree(), currentTicks);
        telemetry.addData("오차", "%.1f°", (targetTicks - currentTicks) / ticksPerDegree());

        telemetry.addLine();
        telemetry.addLine("Y: 터렛 0점 재설정   B: 포즈 재설정");
    }
}
