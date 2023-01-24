package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OurBot;

@Autonomous(name = "BaseAuton")
@Disabled
public class BaseAuton extends LinearOpMode {
    final OurBot robot = new OurBot();



    //gets our run time in case we want to use time
    public final ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initializing Hardware"); //
        telemetry.update();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d, %7d, %7d, %7d",
                robot.leftFront.getCurrentPosition(),
                robot.leftBack.getCurrentPosition(),
                robot.rightFront.getCurrentPosition(),
                robot.rightBack.getCurrentPosition());
        telemetry.update();
    }


    //uses speed and the amount we want to move each side by to move robot
    protected void encoderDrive(double power, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches, double timeout)
    {

        //set up variables
        int leftFrontStart;
        int leftBackStart;
        int rightFrontStart;
        int rightBackStart;

        int leftFrontTarget;
        int leftBackTarget;
        int rightFrontTarget;
        int rightBackTarget;

        if(opModeIsActive())
        {
            //set starting variables
            leftFrontStart = robot.leftFront.getCurrentPosition();
            leftBackStart = robot.leftBack.getCurrentPosition();
            rightFrontStart = robot.rightFront.getCurrentPosition();
            rightBackStart = robot.rightBack.getCurrentPosition();



            // Determine new target position, and pass to motor controller
            leftFrontTarget = leftFrontStart + (int) (leftFrontInches * OurBot.COUNTS_PER_INCH);
            leftBackTarget = leftBackStart + (int) (leftBackInches * OurBot.COUNTS_PER_INCH);
            rightFrontTarget = rightFrontStart + (int) (rightFrontInches * OurBot.COUNTS_PER_INCH);
            rightBackTarget = rightBackStart + (int) (rightBackInches * OurBot.COUNTS_PER_INCH);

            //set the robot's new position that it has to get to
            robot.leftFront.setTargetPosition(leftFrontTarget);
            robot.leftBack.setTargetPosition(leftBackTarget);
            robot.rightFront.setTargetPosition(rightFrontTarget);
            robot.rightBack.setTargetPosition(rightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            power = Math.abs(power);
            robot.leftFront.setPower(power);
            robot.leftBack.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightBack.setPower(power);


            /*

            will keep updating telemetry data until:
            1. opmode is ended by driver
            2. move runs out of time originally set
            3. the robot is in its target position

            */
            while (opModeIsActive() && (runtime.seconds() < timeout) && (robot.leftFront.isBusy()
                    || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()))
            {
                double leftFrontScale = Math.max(1 - ((double)robot.leftFront.getCurrentPosition() - leftFrontStart) / (leftFrontTarget - leftFrontStart), 0.25);
                double leftBackScale = Math.max(1 - ((double)robot.leftBack.getCurrentPosition() - leftBackStart) / (leftBackTarget - leftBackStart), 0.25);
                double rightFrontScale = Math.max(1 - ((double)robot.rightFront.getCurrentPosition() - rightFrontStart) / (rightFrontTarget - rightFrontStart), 0.25);
                double rightBackScale = Math.max(1 - ((double)robot.rightBack.getCurrentPosition() - rightBackStart) / (rightBackTarget - rightBackStart), 0.25);
                robot.leftFront.setPower(power * leftFrontScale);
                robot.leftBack.setPower(power * leftBackScale);
                robot.rightFront.setPower(power * rightFrontScale);
                robot.rightBack.setPower(power * rightBackScale);

                // Display it for the driver.
                telemetry.addData("Running to", "%7d, %7d, %7d, %7d", leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                telemetry.addData("Currently at", "%7d, %7d, %7d, %7d", robot.leftFront.getCurrentPosition(), robot.leftBack.getCurrentPosition(), robot.rightFront.getCurrentPosition(), robot.rightBack.getCurrentPosition());
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