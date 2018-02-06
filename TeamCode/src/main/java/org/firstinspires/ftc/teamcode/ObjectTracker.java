package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.security.Policy;

/**
 * Created by shach on 2/6/2018.
 */
@TeleOp(name="Auto test", group="Linear Opmode")
//@Disabled
public class ObjectTracker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AXDD0ab/////AAAAGVY8yZ0DZkRli/CqkYkxP+x7xWwc26ygyIhnpH1H8tG0U8Ykazr3nnAdbBEIa9sqYl6AIynOF1+YhAsYg8DDElKwceG6jOZcoNBBWPW7aK5wR+3tf4S67yLBZPo1RyfPkl/RVo1mv5YS7Di01lwSZwkT7qaBb/rTz7MKYMrqV5sOiIsaKt6pq5zJev92RQXCaR40uMC6zY9V7dzRssIWK9+4FNKgtrp75tDbFGrVbgc4KTE2nyhl51g6tBB1wZI7wXm7NjcrQd69Gqi2fWYVZhOHwLPDaglMQa8znneo5CrXAULf3df0RSG1wGAsyZixFS17B79lZvtF3RYPuzH4ctHsr3BYFYVBEa70Rz+/5PMH";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS,1);

        VuforiaTrackables crypto = vuforiaLocalizer.loadTrackablesFromAsset("crypto_OT.xml");
        crypto.get(0).setName("crypto");



        waitForStart();
        crypto.activate();

        while (opModeIsActive()){

            for (VuforiaTrackable crypt: crypto){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) crypt.getListener()).getPose();

                if (pose != null){
                    VectorF transletion = pose.getTranslation();


                    telemetry.addData(crypt.getName()+ "-Translation" , transletion);
                    double degreesToTurn = Math.atan2(transletion.get(1), transletion.get(2));
                    telemetry.addData(crypt.getName() + "degrees", degreesToTurn);
                }

            }
            telemetry.update();

        }



    }
}
