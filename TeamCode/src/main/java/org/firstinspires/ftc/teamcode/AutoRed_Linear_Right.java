/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red Autonomous_Right", group="Linear Opmode")
//@Disabled
public class AutoRed_Linear_Right extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    int count;
    HardwareShacharV robot = new HardwareShacharV();

    OpenGLMatrix lastLocation = null;
    double tX;
    double tY;
    double tZ;
    /** ----------**/
    double rX;
    double rY;
    double rZ;
    VuforiaLocalizer vuforia;
    int position = 0;









    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXDD0ab/////AAAAGVY8yZ0DZkRli/CqkYkxP+x7xWwc26ygyIhnpH1H8tG0U8Ykazr3nnAdbBEIa9sqYl6AIynOF1+YhAsYg8DDElKwceG6jOZcoNBBWPW7aK5wR+3tf4S67yLBZPo1RyfPkl/RVo1mv5YS7Di01lwSZwkT7qaBb/rTz7MKYMrqV5sOiIsaKt6pq5zJev92RQXCaR40uMC6zY9V7dzRssIWK9+4FNKgtrp75tDbFGrVbgc4KTE2nyhl51g6tBB1wZI7wXm7NjcrQd69Gqi2fWYVZhOHwLPDaglMQa8znneo5CrXAULf3df0RSG1wGAsyZixFS17B79lZvtF3RYPuzH4ctHsr3BYFYVBEa70Rz+/5PMH";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;



        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);




        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        relicTrackables.activate();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (runtime.seconds() < 10 && position == 0) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    telemetry.addData("VuMark", "%s visible", vuMark);
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));



                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        tX = trans.get(0);
                        tY = trans.get(1);
                        tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        rX = rot.firstAngle;
                        rY = rot.secondAngle;
                        rZ = rot.thirdAngle;
                    }
                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        position = 2;
                        telemetry.addData("Position:", position);
                        telemetry.addData("VuMark is", "CENTER");
                        telemetry.addData("X =", tX);
                        telemetry.addData("y =", tY);
                        telemetry.addData("Z =", tZ);


                    } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                        position = 3;
                        telemetry.addData("Position:", position);
                        telemetry.addData("VuMark is", "LEFT");
                        telemetry.addData("X =", tX);
                        telemetry.addData("y =", tY);
                        telemetry.addData("Z =", tZ);


                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        position = 1;
                        telemetry.addData("Position:", position);
                        telemetry.addData("VuMark is", "RIGHT");
                        telemetry.addData("X =", tX);
                        telemetry.addData("y =", tY);
                        telemetry.addData("Z =", tZ);


                    }

                } else {
                    position = 0;
                    telemetry.addData("Position:", position);
                    telemetry.addData("VuMark", "not visible");

                }
            }

            if(count == 0) {

                robot.S1Motor.setPosition(0.7);
                robot.sleep(1000);
                if (robot.colorSensor.red() > robot.colorSensor.blue()){
                    robot.S2Motor.setPosition(0.7);


                }
                else robot.S2Motor.setPosition(0.3);
                sleep(1000);
                robot.S2Motor.setPosition(0.5);
                robot.S1Motor.setPosition(0.2);
                robot.resetEncoders();

                robot.B2Motor.setPower(0.2);
                robot.B1Motor.setPower(-0.2);
                sleep(2000);
                robot.B1Motor.setPower(0);
                robot.Right(0.3);
                sleep(2000);


                robot.TurnRight(0.3);
                sleep(1440);




                if (position == 1) {
                    telemetry.addData("vuMark", "right");
                    telemetry.update();

                    robot.Right(-0.3);
                    sleep(400);
                }
                else if (position == 2|| position ==0) {
                    telemetry.addData("vuMark", "center");
                    telemetry.update();
                    robot.Right(-0.3);
                    sleep(1300);




                }

                else if (position == 3) {
                    telemetry.addData("vuMark", "left");
                    telemetry.update();
                    robot.Right(-0.3);
                    sleep(1960);

                }
                robot.Stop();
                robot.sleep(3000);
                robot.Forward(0.3);
                sleep(800);
                robot.Stop();
                robot.B2Motor.setPower(0);

                robot.Forward(-0.3);
                sleep(450);
                robot.Forward(0.3);
                sleep(1000);
                robot.Forward(-0.3);
                sleep(300);
                robot.Stop();

                count++;


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


                count++;

            }
            */


            // Setup a variable for each drive wheel to save power level for telemetry

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position:", position);
            telemetry.update();
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
