package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OurBot;


/*
    TODO

    reverse the motor direction so positive power value = forwards motion
    make a secondary set of controls, other way of controlling to see which one is better
    condense code a little bit, make neater
    make a test opmode
    talk about auton



 */

@TeleOp(name="Tank Drive")
public class TankDrive extends LinearOpMode {
    private final OurBot robot = new OurBot();
    private double basePower = 0.3;







    /*
    The main loop for the controller

     */
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Let OurRobot do the heavy lifting of getting and initializing the hardware
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // Run until the end of the match (driver presses STOP)
        while(opModeIsActive())
        {

            /*
            Gamepad 1 controls

            going to first use both joysticks to move

            left joystick moves left side up, right moves right up


             */

            //drive

            //the way motors are config, this is what gets tank drive to work
            //postitive == forward for motor
            //negative == push up on gamepad
            double leftDrive = -(gamepad1.left_stick_y);
            double rightDrive = -(gamepad1.right_stick_y);
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;

            double leftPower = leftDrive * basePower;
            double rightPower = rightDrive * basePower;


            //strafe right
            if (rightTrigger > 0) {

                //left wheels going out
                //right wheels going in
                robot.leftFront.setPower(basePower * rightTrigger);
                robot.leftBack.setPower(-(basePower * rightTrigger));
                robot.rightFront.setPower(-(basePower*rightTrigger));
                robot.rightBack.setPower(basePower*rightTrigger);



            }
            //strafe left
            else if (leftTrigger > 0) {
                //left wheels going in
                //right wheels going out
                robot.leftFront.setPower(-(basePower * leftTrigger));
                robot.leftBack.setPower(basePower * leftTrigger);
                robot.rightFront.setPower(basePower * leftTrigger);
                robot.rightBack.setPower(-(basePower * leftTrigger));

            } else {
                //sets power for main tank drive
                robot.leftFront.setPower(leftPower);
                robot.leftBack.setPower(leftPower);
                robot.rightFront.setPower(rightPower);
                robot.rightBack.setPower(rightPower);

            }


            //updates driver hub
            telemetry.addData("Power", basePower);
            telemetry.update();


        }

    }
}
