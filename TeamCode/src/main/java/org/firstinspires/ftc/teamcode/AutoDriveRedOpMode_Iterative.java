/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */




@Autonomous(name="Auto_Red", group="Iterative Opmode")  // @Tom(...) is the other common choice
@Disabled
public class AutoDriveRedOpMode_Iterative extends OpMode {
    /* Declare OpMode members. */

    private int count = 0;

    //camera//
    OpenGLMatrix lastLocation = null;
    double tX;
    double tY;
    double tZ;
    /** ----------**/
    double rX;
    double rY;
    double rZ;

    VuforiaLocalizer vuforia;


    HardwareShacharV robot = new HardwareShacharV();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXDD0ab/////AAAAGVY8yZ0DZkRli/CqkYkxP+x7xWwc26ygyIhnpH1H8tG0U8Ykazr3nnAdbBEIa9sqYl6AIynOF1+YhAsYg8DDElKwceG6jOZcoNBBWPW7aK5wR+3tf4S67yLBZPo1RyfPkl/RVo1mv5YS7Di01lwSZwkT7qaBb/rTz7MKYMrqV5sOiIsaKt6pq5zJev92RQXCaR40uMC6zY9V7dzRssIWK9+4FNKgtrp75tDbFGrVbgc4KTE2nyhl51g6tBB1wZI7wXm7NjcrQd69Gqi2fWYVZhOHwLPDaglMQa8znneo5CrXAULf3df0RSG1wGAsyZixFS17B79lZvtF3RYPuzH4ctHsr3BYFYVBEa70Rz+/5PMH";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;



        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);


        relicTrackables.activate();



        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        robot.init(hardwareMap);
        //A2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        //A1Motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //A4Motor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");



        //B2

        /*robot.Forward(0.1);
        robot.smartSleep(1);
        robot.Stop();*/


        /*robot.Right(0.35);
        robot.sleep(1170);
        robot.Stop();
        robot.TurnRight(0.36);
        robot.sleep(260);
        robot.Stop();

        robot.Forward(0.35);

        robot.Stop();*/




    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //robot.Forward(1);
        //robot.sleep(5000);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */



    @Override
    public void start() {

        robot.runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + robot.runtime.toString());

        if(count == 0){

            robot.runtime.reset();
            while (robot.runtime.seconds() < 5){
            if (5==5){
                telemetry.addData("vuMark", "right");
            }
            else if (5==5){
                    telemetry.addData("vuMark", "center");
            }
            else if (5==5){
                    telemetry.addData("vuMark", "left");
            }
            telemetry.update();



            }
            /*
            robot.S3Motor.setPosition(0.76);
            robot.sleep(1000);
            if (robot.colorSensor.red() < robot.colorSensor.blue()) {
                robot.TurnRight(0.1);
                robot.sleep(600);
                robot.Stop();
                robot.S3Motor.setPosition(0.2);
                robot.sleep(1000);
                robot.Forward(0.1);
                robot.sleep(450);
                robot.Stop();
                robot.TurnLeft(0.1);
                robot.sleep(600);
                robot.Stop();
            }
            else {
                robot.TurnLeft(0.1);
                robot.sleep(600);
                robot.Stop();
                robot.S3Motor.setPosition(0.2);
                robot.sleep(1000);
                robot.Forward(0.1);
                robot.sleep(450);
                robot.Stop();
                robot.TurnRight(0.1);
                robot.sleep(600);
                robot.Stop();
            }
            */

            count++;

        }



        /*another way to do ^ ,a = Math.pow(a,2);*/
        //regulation of the motor's power values.


        }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }
}
