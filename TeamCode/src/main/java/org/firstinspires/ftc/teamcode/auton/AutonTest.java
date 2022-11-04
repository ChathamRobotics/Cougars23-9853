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
public class AutonTest extends BaseAuton{

    //set distance to one square on the field
    double distance = 24;

    //can fiddle with this to test it properly
    double power = 0.5;


    //max time the move can take
    double timeout = 5;


    int leftFrontStart;
    int leftBackStart;
    int rightFrontStart;
    int rightBackStart;

    int leftFrontTarget;
    int leftBackTarget;
    int rightFrontTarget;
    int rightBackTarget;


    //this method runs runOpMode() in BaseAuton which inits robot and some other things
    public void runOpMode()
    {
        super.runOpMode();
        waitForStart();

        //makes sure opMode is still active
        if(opModeIsActive())
        {
            leftFrontStart = robot.leftFront.getCurrentPosition();
            leftBackStart = robot.leftBack.getCurrentPosition();
            rightFrontStart = robot.rightFront.getCurrentPosition();
            rightBackStart = robot.rightBack.getCurrentPosition();



            // Determine new target position, and pass to motor controller
            leftFrontTarget = leftFrontStart + (int) (distance * OurBot.COUNTS_PER_INCH);
            leftBackTarget = leftBackStart + (int) (distance * OurBot.COUNTS_PER_INCH);
            rightFrontTarget = rightFrontStart + (int) (distance * OurBot.COUNTS_PER_INCH);
            rightBackTarget = rightBackStart + (int) (distance * OurBot.COUNTS_PER_INCH);


            //sets the position the robot will run to with encoderes
            robot.leftFront.setTargetPosition(leftFrontTarget);
            robot.leftBack.setTargetPosition(leftBackTarget);
            robot.rightFront.setTargetPosition(rightFrontTarget);
            robot.rightBack.setTargetPosition(rightBackTarget);


            //makes the robot run to position given power (next step)
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //don't worry about the time stuff
            runtime.reset();
            power = Math.abs(power);
            robot.leftFront.setPower(power);
            robot.leftBack.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightBack.setPower(power);

            //updates the data on the driver hub while the robot is in motion
            while (opModeIsActive() && (runtime.seconds() < timeout) && (robot.leftFront.isBusy()
                    || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()))
            {

                // Display data to see if it is running smoothly
                telemetry.addData("Running to", "%7d, %7d, %7d, %7d", leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                telemetry.addData("Currently at", "%7d, %7d, %7d, %7d", robot.leftFront.getCurrentPosition(), robot.leftBack.getCurrentPosition(), robot.rightFront.getCurrentPosition(), robot.rightBack.getCurrentPosition());
                telemetry.addData("Inches Traveled", "%7d, %7d, %7d, %7d", robot.leftFront.getCurrentPosition()/ OurBot.COUNTS_PER_INCH, robot.leftBack.getCurrentPosition()/ OurBot.COUNTS_PER_INCH, robot.rightFront.getCurrentPosition()/ OurBot.COUNTS_PER_INCH, robot.rightBack.getCurrentPosition()/ OurBot.COUNTS_PER_INCH  );
                telemetry.update();
            }

            // Stop all motion once the loop ends (it reached target position)
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




    }
}
