package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OurBot;

@Autonomous(name = "Basic Auton")
public class BasicAuton extends BaseAuton{

    @Override
    public void runOpMode() {

        //sets up auton by running parent's method
        super.runOpMode();

        //parent class sends message to verify succesful encoder setup

        //wait for driver to press play
        waitForStart();

        encoderDrive(0.3, 20, 20, 20, 20, 5);



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.




    }
}