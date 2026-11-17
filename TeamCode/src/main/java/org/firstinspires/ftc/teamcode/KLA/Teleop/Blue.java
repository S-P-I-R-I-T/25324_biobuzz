package org.firstinspires.ftc.teamcode.KLA.Teleop;

import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.opmode.BulkReadHook;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.KLA.Util.Action;
import org.firstinspires.ftc.teamcode.KLA.Util.Drive;
import org.firstinspires.ftc.teamcode.KLA.Util.Robot;

@NextTeleop(name = "Blue")
public class Blue extends NextOpMode {
    private final Robot robot;
    private Drive drive;

    public Blue(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
    }

    @Override public void start() {
        Scheduler.reset();
        drive = new Drive(hardwareMap, telemetry, gamepad1);
        drive.init();
        robot.action = new Action(hardwareMap);
        robot.mechanisms = java.util.Set.of(robot.action);
        CommandGamepad operator = new CommandGamepad(gamepad2);
        Action action = robot.action;

        // 커맨드가 끝날 때(버튼 해제) 스스로 상태를 해제하므로 onFalse(intakeOff)는 필요 없다.
        operator.a().whileTrue(action.intakeForward());   // 발사 중에 누르면 강제 급탄
        operator.b().whileTrue(action.intakeReverse());
        operator.rightTrigger().isOver(0.5)
                .whileTrue(action.shoot())
                .onFalse(action.stopShooter());
        operator.dpadLeft().onTrue(action.blockerOpen());
        operator.dpadRight().onTrue(action.blockerClose());
        operator.dpadUp().onTrue(action.flowerOpen());
        operator.dpadDown().onTrue(action.flowerClose());
    }

    @Override public void periodic() {
        if (drive != null) drive.controlWithPad();
        Action action = robot.action;
        if (action != null) {
            telemetry.addData("shooter", "%.0f / %.0f  %s",
                    action.shooterVelocity(), Action.SHOOTING_VELOCITY,
                    action.shooterAtSpeed() ? "READY" : "spin-up");
            telemetry.addData("feed", action.isShooting()
                    ? (action.isFeeding() ? "급탄 중 (" + action.feedReason() + ")" : "대기") : "-");
        }
    }
}
