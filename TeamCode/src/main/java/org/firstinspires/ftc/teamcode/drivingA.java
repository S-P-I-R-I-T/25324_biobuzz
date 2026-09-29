package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "drivingA")
public class drivingA extends LinearOpMode {
    DcMotor fl,fr,rl,rr;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class,"imu");
        //가서 컨트롤허브 위치 보고 바꾸기
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
           RevHubOrientationOnRobot.LogoFacingDirection.UP,
           RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        //IMU 초기화
        imu.initialize(parameters);

        waitForStart();
            // Pre-runㅣ

        while (opModeIsActive()) {
                // OpMode loop
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotx = x * Math.cos(-heading) - y * Math.sin(-heading);
            double roty = x * Math.sin(-heading) + y * Math.cos(-heading);

            fl.setPower(roty + rotx + rotate);
            fr.setPower(roty - rotx - rotate);
            rl.setPower(roty - rotx + rotate);
            rr.setPower(roty + rotx - rotate);

            //오류 생기거나 뭔가 이상하면 이걸로 위의 각 모터 셋파워 값 나눠보기
            //Math.max(a,b) -> a,b 중 큰 값으로 변환, 즉 모터 셋파워 값이 1이 안 넘어가게 방지
            //double d = Math.max(Math.abs(rotx)+Math.abs(roty)+Math.abs(rotate),1);
        }
    }
}
