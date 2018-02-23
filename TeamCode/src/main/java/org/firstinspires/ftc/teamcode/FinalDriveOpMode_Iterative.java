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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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




@TeleOp(name="Drive", group="Iterative Opmode")  // @Tom(...) is the other common choice
@Disabled
public class FinalDriveOpMode_Iterative extends OpMode {
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
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
     double     enco    = 1440 ;


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
        robot.A1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.A2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.A3Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.A4Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //B2
        //robot.B2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
        //robot.B2Motor.setTargetPosition(10);
        //robot.B2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //robot.S3Motor.setPosition(0.2);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + robot.runtime.toString());
        //robot.B2Motor.setPower(0.2);

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

        /*robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (Double.parseDouble(robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle)) == 54) {
            telemetry.addData("gyro", " hi");
        }
        */

        /*another way to do ^ ,a = Math.pow(a,2);*/
        //regulation of the motor's power values.


        LeftAxis_y = -c * gamepad1.right_stick_y;
        LeftAxis_x = c * -gamepad1.right_stick_x;
        LeftAxis_z = -r * gamepad1.left_stick_x;


        //calculation of the power for each motor.
        powerA2 = LeftAxis_y + LeftAxis_x + LeftAxis_z;
        powerA1 = -LeftAxis_y + LeftAxis_x + LeftAxis_z;
        powerA3 = LeftAxis_y - LeftAxis_x + LeftAxis_z;
        powerA4 = -LeftAxis_y - LeftAxis_x + LeftAxis_z;

        /*Noramalization of the Motor's power values  */
        MaxPower = Math.max(Math.max(Math.abs(powerA1), Math.abs(powerA2)), Math.max(Math.abs(powerA3), Math.abs(powerA4)));

        if (powerA1 > 1) {
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


        if (gamepad1.a) {
            c = 1;
        }



            if (gamepad1.right_bumper) {
                c = 1;
                r = 0.3;
            } else {
                if (gamepad1.left_bumper) {
                    c = 0.36;
                    r = 0.7;
                } else {
                    c = 0.7;
                    r = 0.5;
                }
            }

            if (MaxPower < 1) {
                MaxPower = 1;
            }

            robot.A1Motor.setPower(powerA1 / MaxPower * c);
            robot.A2Motor.setPower(powerA2 / MaxPower * c);
            robot.A3Motor.setPower(powerA3 / MaxPower * c);
            robot.A4Motor.setPower(powerA4 / MaxPower * c);

            robot.B1Motor.setPower(gamepad2.right_stick_y);
            //robot.B2Motor.setPower(-gamepad1.left_trigger);
/*
            robot.B2Motor.setTargetPosition(1440);
            robot.B2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (robot.B2Motor.getCurrentPosition() < 1440) {
                if (gamepad2.a) {
                    robot.B2Motor.setPower(0.3);
                }
            }
        if (robot.B2Motor.getCurrentPosition() > 0) {
            if (gamepad2.x) {
                robot.B2Motor.setPower(-0.3);
            }
        }
            else {

            robot.B2Motor.setPower(0);
        }
*/







            telemetry.addData("Status", "a = " + a);
            telemetry.addData("Status", "b = " + b);
        telemetry.addData("Status", "B2 = " +robot.B2Motor.getCurrentPosition() );
        telemetry.addData("Status", "A1 = " +robot.A1Motor.getCurrentPosition() );


            telemetry.update();

        }


    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop () {
    }
}
