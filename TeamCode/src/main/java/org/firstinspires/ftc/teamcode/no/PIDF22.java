package org.firstinspires.ftc.teamcode.no;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class PIDF22 extends OpMode {
    public DcMotorEx outtake ;
    Servo hood;
    public static double SHOOTING_VELOCITY = 1300;
    public static double PREHEAT_VELOCITY  = 900;

    private boolean Outtake_wasPressed = false;

    double curTargetVelocity = PREHEAT_VELOCITY;
    public static double F = 0; //15.5
    public static double P = 0;


    // 16 16 / 5 5 / 2100 2100
    //block 0.35
    // push 0.54

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001};

    int stepIndex = 1;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "shooter");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotor.Direction.FORWARD);

        hood = hardwareMap.get(Servo.class,"hood");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");

    }

    @Override
    public void loop() {
        hood.setPosition(0.35);
        if(gamepad1.yWasPressed()) {
            curTargetVelocity = SHOOTING_VELOCITY;

        }
        if(gamepad1.xWasPressed()) {
            curTargetVelocity = PREHEAT_VELOCITY;

        }

        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F*(12/hardwareMap.voltageSensor.iterator().next().getVoltage()));
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set Velocity
        outtake.setVelocity(curTargetVelocity);

        double curVelocity = outtake.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity",curTargetVelocity);
        telemetry.addData("Current Velocity","%.2f",curVelocity);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)",P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)",F);
        telemetry.addData("Step Size","%.4f (B button)",stepSizes[stepIndex]);

        telemetry.update();

    }

}