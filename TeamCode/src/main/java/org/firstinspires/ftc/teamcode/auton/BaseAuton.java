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

@Autonomous(name = "BaseAuton")
public class BaseAuton extends LinearOpMode {
    final OurBot robot = new OurBot();

    //dont know what this does
    //BNO055IMU imu = null;

    //gets our run time in case we want to use time
    private final ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        robot.init(hardwareMap);
    }


    //uses speed and the amount we want to move each side by to move robot
    protected void encoderDrive(double speed, double leftInches, double rightInches)
    {
        int leftFrontStart;
        int leftBackStart;
        int rightFrontStart;
        int rightBackStart;

        int leftFrontTarget;
        int leftBackTarget;
        int rightFrontTarget;
        int rightBackTarget;

        leftFrontStart = robot.leftFront.getCurrentPosition();
        leftBackStart = robot.leftBack.getCurrentPosition();
        rightFrontStart = robot.rightFront.getCurrentPosition();
        rightBackStart = robot.rightBack.getCurrentPosition();
        // Determine new target position, and pass to motor controller
        leftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * OurBot.COUNTS_PER_INCH);
        leftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftInches * OurBot.COUNTS_PER_INCH);
        rightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * OurBot.COUNTS_PER_INCH);
        rightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightInches * OurBot.COUNTS_PER_INCH);
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
        speed = Math.abs(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);

    }

}
