package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OurBot;


@TeleOp
public class MecanumDrive extends LinearOpMode {
    OurBot robot = new OurBot();
    double basePower = 0.5;
    double power;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        waitForStart();
        power = basePower;
        while(opModeIsActive())
        {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.leftFront.setPower(power*(y + x + rx));
            robot.leftBack.setPower(power*(y - x + rx));
            robot.rightFront.setPower(power*(y - x - rx));
            robot.rightBack.setPower(power*(y + x - rx));

            robot.arm.setPower(-gamepad2.left_stick_y * basePower);

            if(gamepad1.triangle)
            {
                power = basePower + 0.3;
            }
            else if(gamepad1.circle)
            {
                power = basePower;
            }
            else if(gamepad1.cross)
            {
                power = basePower - 0.3;
            }

            //controls claw
            if(gamepad2.right_trigger > 0)
            {
                //closed position = 0.82
                robot.claw.setPosition(0.5);
            }else if (gamepad2.left_trigger > 0)
            {
                //open position = 0.53556
                robot.claw.setPosition(0.25);
            }


        }

    }
}
