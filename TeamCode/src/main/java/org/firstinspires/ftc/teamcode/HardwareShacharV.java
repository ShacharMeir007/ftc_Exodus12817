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

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
 public class HardwareShacharV
{
    /* Public OpMode members. */
    static public ElapsedTime runtime = new ElapsedTime();
    static public DcMotor A1Motor= null;
    static public DcMotor A2Motor = null;
    static public DcMotor A3Motor = null;
    static public DcMotor A4Motor = null;
    static public DcMotor B1Motor = null;
    static public DcMotor B2Motor = null;
    //static public DcMotor B1Motor = null;
    //static public DcMotor B2Motor = null;

    static public Servo S1Motor = null;
    static public Servo S2Motor = null;



    static public  ColorSensor colorSensor = null;

    int vuMarkposition= 0;

    //Gyro
    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation angles;
    Acceleration gravity;






    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareShacharV() {

    }


    /* Initialize standard Hardware interfaces */
    public  void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors, Sensors and Vuforia
        A1Motor = hwMap.get(DcMotor.class,"A1_drive");
        A2Motor = hwMap.get(DcMotor.class,"A2_drive");
        A3Motor = hwMap.get(DcMotor.class,"A3_drive");
        A4Motor = hwMap.get(DcMotor.class,"A4_drive");
        B1Motor = hwMap.get(DcMotor.class,"B1");
        B2Motor = hwMap.get(DcMotor.class,"B2");
        S1Motor = hwMap.get(Servo.class,"S1");
        S2Motor = hwMap.get(Servo.class,"S2");


        colorSensor = hwMap.get(ColorSensor.class , "color");

        A1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        B2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        A1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        B2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);










    // Set all motors to zero power
        //C1Motor.setTargetPosition(0);

        //Autonomous period//




        //A1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //A2Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //A3Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //A4Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Define and initialize ALL installed servos.



        A1Motor.setPower(0);
        A2Motor.setPower(0);
        A3Motor.setPower(0);
        A4Motor.setPower(0);


        //color.enableLed(true);
        //B1Motor.setPower(0);
        //B2Motor.setPower(0);
        /*
        S1Motor.setPosition(0.5);
        S2Motor.setPosition(0.4);
        */
        S1Motor.setPosition(0.2);
        sleep(50);
        S2Motor.setPosition(0.4);
        /*
        while(S2Motor.getPosition()!= 0.4){
            double position =S2Motor.getPosition();
            if(S2Motor.getPosition()< 0.4){
                position = position+0.01;
                S2Motor.setPosition(position);
            }
            else {
                position = position-0.01;
                S2Motor.setPosition(position);
            }
            sleep(250);


        }
        */

    }
    static void Stop(){
        A1Motor.setPower(0);
        A2Motor.setPower(0);
        A3Motor.setPower(0);
        A4Motor.setPower(0);
    }
    static void Forward(double power) {
        A1Motor.setPower(-power);
        A2Motor.setPower(power);
        A3Motor.setPower(power);
        A4Motor.setPower(-power);
    }
    static void Right(double power) {
        A1Motor.setPower(-power);
        A2Motor.setPower(-power);
        A3Motor.setPower(power);
        A4Motor.setPower(power);
    }
    static void TurnRight(double power) {
        A1Motor.setPower(-power);
        A2Motor.setPower(-power);
        A3Motor.setPower(-power);
        A4Motor.setPower(-power);
    }



    static void Right (double power, int target){
        //A1Motor.setTargetPosition(target);
        //A2Motor.setTargetPosition(-target);
        //A3Motor.setTargetPosition(target);
        //A4Motor.setTargetPosition(-target);
        power = Math.abs(power);
        final double inc = power/10;
        int count = 1;
        /**
         *
         **/
        if (A3Motor.getCurrentPosition() < target) {
            while (A3Motor.getCurrentPosition() < target) {
                    if (count<=10){
                        power = inc * count;
                    }

                    sleep(100);
                    A1Motor.setPower(-power);
                    A2Motor.setPower(-power);
                    A3Motor.setPower(power);
                    A4Motor.setPower(power);
                    count++;

            }

        }
        else if (A3Motor.getCurrentPosition() > target){
            power = -power;
            count = -count;
            while (A3Motor.getCurrentPosition() > target) {
                if (count>=-10){
                    power = inc * count;
                }

                sleep(100);
                A1Motor.setPower(-power);
                A2Motor.setPower(-power);
                A3Motor.setPower(power);
                A4Motor.setPower(power);
                count--;
            }

        }
        else{}

        A1Motor.setPower(power);
        A2Motor.setPower(power);
        A3Motor.setPower(-power);
        A4Motor.setPower(-power);
        sleep(25);

        A1Motor.setPower(0);
        A2Motor.setPower(0);
        A3Motor.setPower(0);
        A4Motor.setPower(0);



        A1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        A1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    static void Forward (double power, int target){
        //A1Motor.setTargetPosition(target);
        //A2Motor.setTargetPosition(-target);
        //A3Motor.setTargetPosition(target);
        //A4Motor.setTargetPosition(-target);
        power = Math.abs(power);
        final double inc = power/10;
        int count = 1;
        /**
         *
         **/
        if (A3Motor.getCurrentPosition() < target) {
            while (A3Motor.getCurrentPosition() < target) {
                if (count<=10){
                    power = inc * count;
                }

                sleep(100);
                A1Motor.setPower(power);
                A2Motor.setPower(-power);
                A3Motor.setPower(-power);
                A4Motor.setPower(power);
                count++;

            }

        }
        else if (A3Motor.getCurrentPosition() > target){
            power = -power;
            count = -count;
            while (A3Motor.getCurrentPosition() > target) {
                if (count>=-10){
                    power = inc * count;
                }

                sleep(100);
                A1Motor.setPower(power);
                A2Motor.setPower(-power);
                A3Motor.setPower(-power);
                A4Motor.setPower(power);
                count--;
            }

        }
        else{}

        A1Motor.setPower(power);
        A2Motor.setPower(power);
        A3Motor.setPower(-power);
        A4Motor.setPower(-power);
        sleep(25);

        A1Motor.setPower(0);
        A2Motor.setPower(0);
        A3Motor.setPower(0);
        A4Motor.setPower(0);



        A1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        A1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    /**
    static void Left (double power, int target){
        A1Motor.setTargetPosition(target);
        A2Motor.setTargetPosition(target);
        A3Motor.setTargetPosition(target);
        A4Motor.setTargetPosition(target);

        A1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        A2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        A3Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        A4Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /**
         *

        A1Motor.setPower(-power);
        A2Motor.setPower(power);
        A3Motor.setPower(-power);
        A4Motor.setPower(power);
        /**
         *

        A1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }**/
    /**
    static void TurnLeft(double power, int target){
        A1Motor.setTargetPosition(target);
        A2Motor.setTargetPosition(target);
        A3Motor.setTargetPosition(target);
        A4Motor.setTargetPosition(target);

        A1Motor.setPower(power);
        A2Motor.setPower(power);
        A3Motor.setPower(-power);
        A4Motor.setPower(-power);
    }
    static void TurnRight(double power, int target){
        A1Motor.setTargetPosition(target);
        A2Motor.setTargetPosition(target);
        A3Motor.setTargetPosition(target);
        A4Motor.setTargetPosition(target);

        A1Motor.setPower(-power);
        A2Motor.setPower(-power);
        A3Motor.setPower(power);
        A4Motor.setPower(power);
    }
     */
    public static final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    private volatile boolean   isStarted       = false;
    private volatile boolean   stopRequested   = false;

    public final boolean isStarted() {
        return this.isStarted || Thread.currentThread().isInterrupted();
    }
    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }
    public final boolean opModeIsActive() {
        boolean isActive = !this.isStopRequested() && this.isStarted();
        if (isActive) {
            idle();
        }
        return isActive;
    }
    public Telemetry telemetry;
    public void smartSleep(double seconds){
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


     public void resetEncoders(){
        A1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A3Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        A4Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         A1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         A2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         A3Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         A4Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

     }
     public void setTarget(int target){
         A1Motor.setTargetPosition(target);
         A2Motor.setTargetPosition(target);
         A3Motor.setTargetPosition(target);
         A4Motor.setTargetPosition(target);
     }




}

