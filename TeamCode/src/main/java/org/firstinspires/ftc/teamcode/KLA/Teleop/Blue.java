package org.firstinspires.ftc.teamcode.KLA.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.KLA.Util.Action;
import org.firstinspires.ftc.teamcode.KLA.Util.Drive;

@TeleOp()
public class Blue extends LinearOpMode {

    Drive drive;
    Action action;

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap, telemetry, gamepad1);
        drive.init();
        action = new Action(hardwareMap, telemetry);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                action.clearBulkCache();
                action.updateShooterPidf();
                drive.controlWithPad();

                Intake();
                Outtake();
                blocker();
                flower();

            }
        }
    }

    void Intake(){
        if (gamepad2.a){
            action.IntakeOn();
        }else if(gamepad2.b) {
            action.IntakeROn();
        }else{
            action.IntakeOff();
        }
    }

    void Outtake(){
        if (gamepad2.right_trigger > 0.5) {
            action.startShooter();
        } else {
            action.stopShooter();
        }
    }

    void blocker(){
        if (gamepad2.dpad_left){
            action.BlockerOpen();
        }else if (gamepad2.dpad_right){
            action.BlockerClose();
        }
    }

    void flower(){
        if (gamepad2.dpad_up){
            action.FlowerOpen();
        }else if(gamepad2.dpad_down){
            action.FlowerClose();
        }
    }

}
