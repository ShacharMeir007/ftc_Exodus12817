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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="FirstOpMode", group="Iterative Opmode")  // @Tom(...) is the other common choice
@Disabled
public class MainOpMode_Iterative extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor A1Motor= null;
    private DcMotor A2Motor = null;
    private DcMotor A3Motor = null;
    private DcMotor A4Motor = null;
    double LeftAxis_y;
    double LeftAxis_x;
    int ModeA = 1;

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
        A2Motor = hardwareMap.get(DcMotor.class,"A2_drive");
        A1Motor = hardwareMap.get(DcMotor.class,"A1_drive");
        A3Motor = hardwareMap.get(DcMotor.class,"A3_drive");
        A4Motor = hardwareMap.get(DcMotor.class,"A4_drive");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
         A1Motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
          A4Motor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");

    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
       sleep(3000);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        LeftAxis_y = -gamepad1.left_stick_y;
        LeftAxis_x = -gamepad1.left_stick_x;
        if (gamepad1.a){
            ModeA = ModeA+1;
        }

        if(ModeA % 2 == 1){
            if (LeftAxis_y>0 && LeftAxis_x>0 || LeftAxis_y<0 && LeftAxis_x<0){
                A1Motor.setPower(0.5*(LeftAxis_y - LeftAxis_x));
                A3Motor.setPower(0.5*(LeftAxis_y - LeftAxis_x));
                A4Motor.setPower(0.5*(LeftAxis_y));
                A2Motor.setPower(0.5*(LeftAxis_y));
            }
            else {
                A1Motor.setPower(0.5*(LeftAxis_y));
                A3Motor.setPower(0.5*(LeftAxis_y));
                A4Motor.setPower(0.5*(LeftAxis_y + LeftAxis_x));
                A2Motor.setPower(0.5*(LeftAxis_y + LeftAxis_x));
            }
        }
        else{
            if (LeftAxis_y>0 && LeftAxis_x>0 || LeftAxis_y<0 && LeftAxis_x<0) {
                A1Motor.setPower(LeftAxis_y - LeftAxis_x);
                A3Motor.setPower(LeftAxis_y - LeftAxis_x);
                A4Motor.setPower(LeftAxis_y);
                A2Motor.setPower(LeftAxis_y);
            }
            else {
                A1Motor.setPower(LeftAxis_y);
                A3Motor.setPower(LeftAxis_y);
                A4Motor.setPower(LeftAxis_y + LeftAxis_x);
                A2Motor.setPower(LeftAxis_y + LeftAxis_x);
            }
        }
    }

}
