package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OurBot;


@TeleOp(name="Main Drive")
public class MainDrive extends LinearOpMode{
    private final OurBot robot = new OurBot();
    private final double basePower = 0.3;

    public void runOpMode()
    {
        double power = basePower;
        double lowPower = basePower - 0.2;
        double highPower = basePower + 0.2;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Let OurRobot do the heavy lifting of getting and initializing the hardware
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while(opModeIsActive())
        {
            if (gamepad1.dpad_down) {
                power = lowPower;
            }
            if (gamepad1.dpad_up) {
                power = highPower;
            }
            double drive = -(gamepad1.left_stick_y);
            double turn = gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn, -1, 1) * power;
            double rightPower = Range.clip(drive - turn, -1, 1) * power;
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;

            // Strafe right
            if (rightTrigger > 0) {

                // Left wheels going out
                // Light wheels going in
                robot.leftFront.setPower(basePower * rightTrigger);
                robot.leftBack.setPower(-(basePower * rightTrigger));
                robot.rightFront.setPower(-(basePower * rightTrigger));
                robot.rightBack.setPower(basePower * rightTrigger);


            }
            // Strafe left
            else if (leftTrigger > 0) {
                // Left wheels going in
                // Right wheels going out
                robot.leftFront.setPower(-(basePower * leftTrigger));
                robot.leftBack.setPower(basePower * leftTrigger);
                robot.rightFront.setPower(basePower * leftTrigger);
                robot.rightBack.setPower(-(basePower * leftTrigger));

            } else {
                // Sets power for main drive
                robot.leftFront.setPower(leftPower);
                robot.leftBack.setPower(leftPower);
                robot.rightFront.setPower(rightPower);
                robot.rightBack.setPower(rightPower);

            }

            // Sets the power for the robot
            robot.leftFront.setPower(leftPower);
            robot.leftBack.setPower(leftPower);
            robot.rightFront.setPower(rightPower);
            robot.rightBack.setPower(rightPower);


            // Updates telemetry
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Right Stick x", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}