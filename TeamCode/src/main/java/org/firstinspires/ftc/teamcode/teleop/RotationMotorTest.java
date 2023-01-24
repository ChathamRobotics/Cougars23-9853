package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class RotationMotorTest extends LinearOpMode {
    public DcMotor rotation1;
    public DcMotor rotation2;
    public DcMotor arm;
    public Servo claw = null;
    public boolean dual = true;

    //1 = left motor ---- 2 = right motor
    int rotation2Target;
    int rotation1Target;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");

        rotation1 = hardwareMap.get(DcMotor.class, "leftRotation");
        rotation1.setDirection(DcMotorSimple.Direction.FORWARD);
        rotation1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rotation1Target = rotation1.getCurrentPosition();
        //rotation1.setTargetPosition(rotation1Target);
        rotation1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotation1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // rotation1.setPower(0.3);

        rotation2 = hardwareMap.get(DcMotor.class, "rightRotation");
        rotation2.setDirection(DcMotorSimple.Direction.FORWARD);
        rotation2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rotation2Target  = rotation2.getCurrentPosition();
        //rotation2.setTargetPosition(rotation2Target);
        rotation2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotation2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rotation2.setPower(0.3);



        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);









        telemetry.addData("Status", "Initialized");
        telemetry.update();




        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.y)
            {
                rotation1Target = -986;
                rotation2Target = -986;
            }else if(gamepad1.a)
            {
                rotation1Target = -100;
                rotation2Target = -100;
            }



            /*if(dual)
            {
                leftRotation.setPower(-gamepad1.left_stick_y);
                rightRotation.setPower(-gamepad1.left_stick_y);

            }else{
                if(currentMotor == 1){
                    leftRotation.setPower(-gamepad1.left_stick_y);

                }else{
                    rightRotation.setPower(-gamepad1.left_stick_y);
                }
            }*/

           ;

            rotation1.setPower(gamepad1.right_stick_y * 0.5);
            rotation2.setPower(gamepad1.right_stick_y * 0.5);

            arm.setPower(gamepad1.left_stick_y * 0.5);


            //rotation up to high = -986

            //telemetry.addData("Arm Motor Position", arm.getCurrentPosition());

            //rotation1.setTargetPosition(rotation1Target);
            //rotation2.setTargetPosition(rotation2Target);

            if(gamepad1.right_trigger > 0)
            {
                //closed position = 0.82
                claw.setPosition(0.5);
            }else if (gamepad1.left_trigger > 0)
            {
                //open position = 0.53556
                claw.setPosition(0.25);
            }

            telemetry.addData("Arm position", arm.getCurrentPosition());
            telemetry.addData("Right rotation motor position", rotation2.getCurrentPosition());
            telemetry.addData("Left rotation motor position", rotation1.getCurrentPosition());
            telemetry.update();


        }








    }
}
