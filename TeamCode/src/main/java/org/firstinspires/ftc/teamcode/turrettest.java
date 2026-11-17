package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "turrettest")
public class turrettest extends LinearOpMode {
    private static final double MANUAL_POWER = 0.2;
    private static final int HOME_POSITION = 0;

    private DcMotor turret;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotor.class,"turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("D-pad left/right: manual | A: move to home (0)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // D-pad manual input takes priority over the preset target command.
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                if (turret.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (gamepad1.dpad_left) {
                    turret.setPower(-MANUAL_POWER);
                } else {
                    turret.setPower(MANUAL_POWER);
                }
            } else if (gamepad1.a) {
                // RUN_TO_POSITION requires a positive power; direction is chosen by the controller.
                turret.setTargetPosition(HOME_POSITION);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(MANUAL_POWER);
            } else if (gamepad1.b){
                turret.setTargetPosition(-400);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(MANUAL_POWER);
            } else if(turret.getMode() == DcMotor.RunMode.RUN_USING_ENCODER){
                turret.setPower(0);
            }

            telemetry.addData("Position", turret.getCurrentPosition());
            telemetry.addData("Target", turret.getTargetPosition());
            telemetry.addData("Mode", turret.getMode());
            telemetry.update();
        }

        turret.setPower(0);
    }
}
