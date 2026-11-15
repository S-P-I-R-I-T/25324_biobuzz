package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "flower")
public class flowertest extends LinearOpMode {
    Servo blocker;
    public static double pos = 0.5;

    // 0.98 intake
    // 0.5 init

    @Override
    public void runOpMode() {
        waitForStart();
        blocker=hardwareMap.get(Servo.class,"flower");
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                blocker.setPosition(pos);

            }
        }
    }
}