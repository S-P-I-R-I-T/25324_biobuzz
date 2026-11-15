package org.firstinspires.ftc.teamcode.KLA.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.KLA.Util.Action;
import org.firstinspires.ftc.teamcode.KLA.Util.Drive;

@Configurable
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

            }
        }
    }

    void Intake(){
        if (gamepad2.a){
            action.BlockerClose();
            action.IntakeOn();
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
        if (gamepad2.b){
            action.BlockerOpen();
        }else if (gamepad2.x){
            action.BlockerClose();
        }
    }
}
