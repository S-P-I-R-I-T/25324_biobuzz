package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "driving")
public class driving extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor flMotor = hardwareMap.dcMotor.get("fl");
        DcMotor frMotor = hardwareMap.dcMotor.get("fr");
        DcMotor rlMotor = hardwareMap.dcMotor.get("rl");
        DcMotor rrMotor = hardwareMap.dcMotor.get("rr");
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                flMotor.setPower(1);
                frMotor.setPower(1);
                rlMotor.setPower(1);
                rrMotor.setPower(1);
            }
        }
    }
}