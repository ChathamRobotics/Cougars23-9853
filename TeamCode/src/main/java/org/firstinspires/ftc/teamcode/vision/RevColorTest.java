package org.firstinspires.ftc.teamcode.vision;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;





@TeleOp(name="Rev Color Sensor", group = "Test")
public class RevColorTest extends LinearOpMode {
    private NormalizedColorSensor colorSensor;



    //store hsv values
    float hsvValues[] = {0F,0F,0F};

    public void runOpMode()
    {
        //get color sensor from hardware map
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        //sets gain, gain helps scale the raw values based on high/low light
        colorSensor.setGain(2);

        //wait for driver to press start
        waitForStart();

        while(opModeIsActive())
        {

            // convert the RGB values to HSV values.

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);



            //get values from color sensor
            telemetry.addData("Red  ", colors.red);
            telemetry.addData("Green", colors.green);
            telemetry.addData("Blue ", colors.blue);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Saturation", hsvValues[1]);
            telemetry.addData("Value", hsvValues[2]);




            //update telemetry
            telemetry.update();
        }
    }

}
