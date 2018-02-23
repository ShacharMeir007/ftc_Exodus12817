package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by shach on 2/9/2018.
 */
@TeleOp(name="Auto Object", group="Linear Opmode")
@Disabled

public class Auto_ObjectEncoders extends LinearOpMode {
    HardwareShacharV robot = new HardwareShacharV();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        try {
            robot.Tra.runOpMode();
            telemetry.addData("ffd","ff");
            telemetry.update();
        }
        catch (Exception exp){
            telemetry.addData("fuck",exp);
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){





        }






    }
}
