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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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




@Autonomous(name="Auto_Blue", group="Iterative Opmode")  // @Tom(...) is the other common choice
//@Disabled
public class AutoDriveBlueOpMode_Iterative extends OpMode {
    /* Declare OpMode members. */
    private double LeftAxis_y;
    private double LeftAxis_x;
    private double LeftAxis_z;
    private double powerA1;
    private double powerA2;
    private double powerA3;
    private double powerA4;
    private double MaxPower;
    private double a = 0.5;
    private double b = 0.5;
    private double c = 1;
    private double r = 1;
    private double count = 0;




    HardwareShacharV robot = new HardwareShacharV();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

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
        robot.S1Motor.setPosition(0.2);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + robot.runtime.toString());

        if(count == 0) {
            robot.S1Motor.setPosition(0.76);
            robot.sleep(1000);
            if (robot.colorSensor.red() > robot.colorSensor.blue()) {
                robot.TurnRight(0.1,5);
                robot.sleep(600);
                robot.Stop();
                robot.S1Motor.setPosition(0.2);
                robot.sleep(1000);
                robot.Forward(0.1,5);
                robot.sleep(450);
                robot.Stop();
            } else {
                robot.TurnLeft(0.1,5);
                robot.sleep(600);
                robot.Stop();
                robot.S1Motor.setPosition(0.2);
                robot.sleep(1000);
                robot.Forward(0.1,5);
                robot.sleep(450);
                robot.Stop();

            }
            count++;
        }
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)




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
