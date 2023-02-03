package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    private TrajectoryFollower follower;
    public DcMotor leftRotation = null;
    public DcMotor rightRotation = null;
    public DcMotor arm = null;
    public Servo claw = null;

    double basePower = 0.5;
    double power;
    boolean backwards = false;
    int rotationRightTarget;
    int rotationLeftTarget;

    int lowPositionAuton = -710;
    int midPositionAuton = -640;
    int highPositionAuton = 0;

    public Servo clawRotation;

    double scale = 0.75;



    //have to check these
    int lowPositionNormal = 50;
    int midPositionNormal = 150;
    int highPositionNormal = 950;

    int lowPosition = lowPositionAuton;
    int highPosition = highPositionAuton;
    @Override
    public void runOpMode() throws InterruptedException {

        //get mec drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );



            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
