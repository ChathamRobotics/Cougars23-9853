package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OurBot;


@Autonomous
public class RedRight extends BaseAutonOpenCVWebcam {
    int zone;
    final int ARM_HOVER_INCHES = 6;
    int greenAverage, redAverage, blueAverage;

    DistanceSensor distanceSensor;


    public void runOpMode()
    {


        //sets up webcam and robot from parent class
        super.runOpMode();
        /*distanceSensor = hardwareMap.get(DistanceSensor.class, "" +
                "" +
                "" +
                "distanceSensor");*/

        //wait for driver to press start
        telemetry.addData(">", "press play pls");
        telemetry.update();
        waitForStart();







        while(pipeline.getRedAverage() == 0 )
        {
            telemetry.addData("Status", "not working");
            telemetry.update();
            sleep(5);
        }

        telemetry.addData("Status", "it worked!");
        telemetry.update();



        //if blue is the color
        if ( pipeline.getBlueAverage() > pipeline.getGreenAverage() && pipeline.getBlueAverage() > pipeline.getRedAverage())
        {
            telemetry.addData("Color", "Blue");
            zone = 1;
        }

        //if color is green
        else if (pipeline.getGreenAverage() > pipeline.getBlueAverage() && pipeline.getGreenAverage() > pipeline.getRedAverage())
        {
            telemetry.addData("Color", "Green");
            zone = 2;
        }
        else{
            telemetry.addData("Color", "Red");
            zone = 3;
        }
        telemetry.update();





        //grab cone
        robot.claw.setPosition(0.5);
        sleep(100);

        //strafe to low junction
        encoderDrive(0.3, -13.5, 13.5, 13.5, -13.5, 10, true);



        robot.arm.setTargetPosition(550);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.arm.setPower(0.3);
        while (opModeIsActive() &&  robot.arm.isBusy())
        {



            // Display it for the driver.

            telemetry.addData("Currently at", "%7d", robot.arm.getCurrentPosition());
            telemetry.update();
        }
        robot.arm.setPower(0.04);
        encoderDrive(0.3, 6.0, 6.0, 6.0, 6.0, 10, true);


        sleep(200);
        robot.arm.setTargetPosition(0);
        robot.arm.setPower(0.1);
        runtime.reset();
        while (opModeIsActive() &&  robot.arm.isBusy() && (runtime.seconds() < 10))
        {



            // Display it for the driver.

            telemetry.addData("Currently at", "%7d", robot.arm.getCurrentPosition());
            telemetry.update();
        }

        //turns off arm and opens claw
        robot.arm.setPower(0);
        robot.claw.setPosition(0.25);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        switch(zone)
        {
            case(1):
                //move back, left, and up
                encoderDrive(0.3, -6, -6, -6, -6, 10, true);
                encoderDrive(0.3, -10, 10, 10, -10, 10, true);
                encoderDrive(0.3, 26, 26, 26, 26, 10, true);
                break;
            case(2):
                //move back, right, and up
                encoderDrive(0.3, -6, -6, -6, -6, 10, true);
                encoderDrive(0.3, 10.375, -10.375, -10.375, 10.375, 10, true);
                encoderDrive(0.3, 26, 26, 26, 26, 10, true);
                break;
            case(3):
                //move back, far right, and up
                encoderDrive(0.3, -6, -6, -6, -6, 10, true);
                encoderDrive(0.3, 36, -36, -36, 36, 10, true);
                encoderDrive(0.3, 26, 26, 26, 26, 10, true);
                break;
            default:
                break;
        }





        telemetry.update();
        sleep(4000);



    }

}
