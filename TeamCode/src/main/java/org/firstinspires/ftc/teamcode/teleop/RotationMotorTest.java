package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotationMotorTest extends LinearOpMode {
    public DcMotor leftRotation;
    public DcMotor rightRotation;
    public boolean dual = true;

    //1 = left motor ---- 2 = right motor
    public int currentMotor = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        leftRotation = hardwareMap.get(DcMotor.class, "leftFront");
        leftRotation.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRotation.setPower(0);
        leftRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightRotation = hardwareMap.get(DcMotor.class, "leftBack");
        rightRotation.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRotation.setPower(0);
        rightRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.y)
            {
                dual = true;
            }else if(gamepad1.a)
            {
                dual = false;
            }

            if(gamepad1.x)
            {
                currentMotor = 1;
            }else if(gamepad1.b)
            {
                currentMotor = 2;
            }

            if(dual)
            {
                leftRotation.setPower(-gamepad1.left_stick_y);
                rightRotation.setPower(-gamepad1.left_stick_y);

            }else{
                if(currentMotor == 1){
                    leftRotation.setPower(-gamepad1.left_stick_y);

                }else{
                    rightRotation.setPower(-gamepad1.left_stick_y);
                }
            }

            telemetry.addData("Dual", dual );
            telemetry.addData("Current Motor", currentMotor);
            telemetry.update();


        }








    }
}
