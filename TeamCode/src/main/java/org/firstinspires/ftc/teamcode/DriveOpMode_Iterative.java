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




@TeleOp(name="ArcadeDrive", group="Iterative Opmode")  // @Tom(...) is the other common choice
@Disabled
public class DriveOpMode_Iterative extends OpMode {
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
    private double count;



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

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)



        //  This is Tom's Speed System and were testing the robot without it
        /*
        if (gamepad1.a){
            modea++;
            if (modea%2==0){
                LeftAxis_y = 0.5 * gamepad1.left_stick_y;
                LeftAxis_x = 0.5 * -gamepad1.left_stick_x;
                robot.sleep(50);
            }
            else{
                LeftAxis_y = 0.25 * gamepad1.left_stick_y;
                LeftAxis_x = 0.25 * -gamepad1.left_stick_x;
                robot.sleep(50);
            }
        }


        if (gamepad1.x){
            modex++;
            if (modex%2==0){
                LeftAxis_z = -0.36 * gamepad1.right_stick_x;
                robot.sleep(50);
            }
            else{
                LeftAxis_z = -0.5 * gamepad1.right_stick_x;
                robot.sleep(50);
            }
        }*/

        /*another way to do ^ ,a = Math.pow(a,2);*/
        //regulation of the motor's power values.
        LeftAxis_y = -c * gamepad1.left_stick_y * Math.abs(Math.pow(gamepad1.left_stick_y,3));
        LeftAxis_x = c * -gamepad1.left_stick_x* Math.abs(Math.pow(gamepad1.left_stick_x,3));
        LeftAxis_z = -r * gamepad1.right_stick_x* Math.abs(gamepad1.right_stick_x);


        //calculation of the power for each motor.
        powerA2 = LeftAxis_y + LeftAxis_x + LeftAxis_z;
        powerA1 = -LeftAxis_y + LeftAxis_x + LeftAxis_z;
        powerA3 = -LeftAxis_y - LeftAxis_x + LeftAxis_z;
        powerA4 = LeftAxis_y - LeftAxis_x + LeftAxis_z;

        /*Noramalization of the Motor's power values  */
        MaxPower = Math.max(Math.max(Math.abs(powerA1), Math.abs(powerA2)), Math.max(Math.abs(powerA3), Math.abs(powerA4)));

        /*if (powerA1 > 1) {
            powerA1 = powerA1 / MaxPower;
        }
        if (powerA2 > 1) {
            powerA2 = powerA2 / MaxPower;
        }
        if (powerA3 > 1) {
            powerA3 = powerA3 / MaxPower;
        }
        if (powerA4 > 1) {
            powerA4 = powerA4 / MaxPower;
        }


        if(gamepad1.a){
            c=1;
        }
        if(gamepad1.x){
            c=0.5;
        }*/

        /*if(gamepad1.x){
            if(count%2==0){
                c = 1;
            }
            else {
                c = 0.25;
            }
            count++;
        }*/

        if(gamepad1.right_bumper){
            c=1;
            r=0.3;
        }
        else {
            if (gamepad1.left_bumper) {
                c = 0.36;
                r = 0.7;
            }
            else {
                c=0.5;
                r=0.5;
            }
        }

        if(MaxPower<1){
            MaxPower=1;
        }


        robot.A1Motor.setPower(powerA1/MaxPower*c);
        robot.A2Motor.setPower(powerA2/MaxPower*c);
        robot.A3Motor.setPower(powerA3/MaxPower*c);
        robot.A4Motor.setPower(powerA4/MaxPower*c);




        //robot.B2Motor.setPower(gamepad1.right_trigger);
        //robot.B2Motor.setPower(-gamepad1.left_trigger);
        //the movement of the servo motor
        if (gamepad2.left_bumper) {
            if (a <= 0.7) {
                a = a + 0.01;
                robot.sleep(50);
            }
            robot.S1Motor.setPosition(a);
            robot.S2Motor.setPosition(0.9-a);
            robot.sleep(10);
        }

        if (gamepad2.right_bumper) {
            if (a >=0.01) {
                a = a - 0.01;
                robot.sleep(10);

            }
            robot.S1Motor.setPosition(a);
            robot.S2Motor.setPosition(0.9-a);
            robot.sleep(50);

        }
        /*b = robot.S1Motor.getPosition();
        if (a!=b) {
            robot.S1Motor.setPosition(b);
            robot.S2Motor.setPosition(0.9 - b);
            robot.sleep(50);
            a=b;
        }
        else if(a ==b) {
            robot.S1Motor.setPosition(a);
            robot.S2Motor.setPosition(0.9-a);
            robot.sleep(50);

        }
        */
        //quick opening and closing of the servo's
        if(gamepad2.y){
            a=0.71;
            robot.S1Motor.setPosition(0.71);
            robot.S2Motor.setPosition(0.9-0.71);
        }
        if(gamepad2.b){
            a=0.51;
            robot.S1Motor.setPosition(0.51);
            robot.S2Motor.setPosition(0.9-0.51);
        }
        if(gamepad2.x){
            a=0.41;
            robot.S1Motor.setPosition(0.41);
            robot.S2Motor.setPosition(0.9-0.41);
        }




        telemetry.addData("Status","S1 = "+robot.S1Motor.getPosition());
        telemetry.addData("Status","S2 = "+robot.S2Motor.getPosition());
        telemetry.addData("Status","a = "+ a);
        telemetry.addData("Status","b = "+ b);
        telemetry.update();

        }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }
}
