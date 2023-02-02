package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public DcMotor leftRotation = null;
    public DcMotor rightRotation = null;
    public Servo clawRotation;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        leftRotation = hardwareMap.get(DcMotor.class, "leftRotation");
        rightRotation = hardwareMap.get(DcMotor.class, "rightRotation");
        leftRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRotation.setTargetPosition(leftRotation.getCurrentPosition());
        rightRotation.setTargetPosition(rightRotation.getCurrentPosition());
        leftRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRotation.setPower(0.4);
        rightRotation.setPower(0.4);
        clawRotation.setPosition(0.45);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
