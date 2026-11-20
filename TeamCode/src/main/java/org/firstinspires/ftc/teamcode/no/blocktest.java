package org.firstinspires.ftc.teamcode.no;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Disabled
@TeleOp(name = "blocktest")
public class blocktest extends LinearOpMode {
    Servo blocker;
    public static double pos = 0.5;

    // 0.4 open
    // 0.25 close

    @Override
    public void runOpMode() {
        waitForStart();
        blocker=hardwareMap.get(Servo.class,"blocker");
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                blocker.setPosition(pos);

            }
        }
    }
}