package org.firstinspires.ftc.teamcode.teleop;


import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Build;


import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.Buffer;

@TeleOp(name="Webcam picture test")
public class WebcamPicture extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "AWA18Wv/////AAABmdpxJaFs60v9oHAWHu046k0TYI540KMM8arHrZ7oaq/mHJpGMWourIeuTJef6mY+1PiSavuMKEY6nHOe1wxCZ7fGYhXHD6R/khIVWzZkmod+67ZU6lgHdIevIiLQuSWpxXyZQpjkkDcYvgQ5szC4FufvCMBnR/4sxrsmY5Th63en8y/4DqNPO9As8v8tPntZjtFoOGa8hvKd0z4pt8lEXjG2KAojQFU2DaQWEdRI15D/sndCPmoCWSgJ5pABYNzvwNlcE6BCt4cjpgGDKxPifI2Ik3jO0O/grVMRYjHcAJ0uK3R1aw+7n2oEIjkSUQF+r7FDs71u8fIsUv12ZzCa05IQ/bVw8GRoYI1taFH06ON1";

    private VuforiaLocalizer vuforia;

    private VuforiaLocalizer.CloseableFrame frame = null;

    private Image rgb;



    public void runOpMode() throws InterruptedException {
        initVuforia();

        waitForStart();


        while(opModeIsActive())
        {
            if(opModeIsActive()) {
                frame = vuforia.getFrameQueue().take();
                rgb = frame.getImage(0);

                Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                //if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                //Color color = bm.getColor(100, 100);



                //}
                telemetry.addData("Width", rgb.getWidth());
                telemetry.addData("Height", rgb.getHeight());
                telemetry.addData("pixel", bm.getPixel(100, 100)  == Color.BLUE);
                ;
                telemetry.addData("# of frames", frame.getNumImages());
                telemetry.update();
            }
        }

    }

    public void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

    }

}
