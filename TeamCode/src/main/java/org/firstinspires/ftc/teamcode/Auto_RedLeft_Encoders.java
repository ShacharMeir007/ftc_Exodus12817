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
 * Created by shach on 2/11/2018.
 */
@Autonomous(name="Auto_RedLeft_Encoders", group="Linear Opmode")
//@Disabled

public class Auto_RedLeft_Encoders extends LinearOpMode {


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
    public void runOpMode(){
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
        while (opModeIsActive()) {

            while (runtime.seconds() < 5 && position == 0) {
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
                robot.V1Motor.setPower(-0.2);
                robot.V2Motor.setPower(0.2);

                robot.B1Motor.setPower(-0.2);
                sleep(2000);
                robot.V1Motor.setPower(0);
                robot.V2Motor.setPower(0);
                robot.B1Motor.setPower(0);




                telemetry.addData("A1", robot.A1Motor.getCurrentPosition());
                telemetry.addData("A2", robot.A2Motor.getCurrentPosition());
                telemetry.addData("A3", robot.A3Motor.getCurrentPosition());
                telemetry.addData("A4", robot.A4Motor.getCurrentPosition());
                telemetry.update();
                sleep(400);

                //right
                if (position == 1) {
                    telemetry.addData("vuMark", "right");
                    telemetry.update();

                    robot.Right(0.3, 1600);
                    robot.resetEncoders();
                    robot.TurnRight(0.4,2250);
                    sleep(400);



                }
                //center
                else if (position == 2|| position ==0) {
                    telemetry.addData("vuMark", "center");
                    telemetry.update();

                    robot.Right(0.3, 2200);
                    robot.resetEncoders();
                    robot.TurnRight(0.4,2250);
                    sleep(2000);


                }
                //left

                else if (position == 3) {
                    telemetry.addData("vuMark", "left");
                    telemetry.update();
                    robot.Right(0.3, 2750);
                    robot.resetEncoders();
                    robot.TurnRight(0.4,2250);
                    sleep(1525);

                }
                robot.Stop();
                robot.V1Motor.setPower(0.2);
                robot.V2Motor.setPower(-0.2);
                robot.Stop();
                robot.sleep(3000);
                robot.Forward(0.3);
                sleep(1500);
                robot.V1Motor.setPower(0);
                robot.V2Motor.setPower(0);
                robot.Forward(0.3);
                sleep(1500);
                robot.resetEncoders();
                robot.Forward(-0.3, -150);
                sleep(300);
                robot.resetEncoders();

                robot.Stop();
                









                count++;

            }
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
