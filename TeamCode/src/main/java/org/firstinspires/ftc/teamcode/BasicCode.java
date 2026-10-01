package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BasicCode", group = "JR")
public class BasicCode extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, intaketest;

    @Override
    public void runOpMode() {

        // --- Hardware Map ---
        frontLeft  = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BL");
        backRight  = hardwareMap.get(DcMotorEx.class, "BR");
        intaketest = hardwareMap.get(DcMotorEx.class, "IT");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;

            double max = Math.max(1.0,
                    Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(backLeftPower),
                                    Math.max(Math.abs(frontRightPower),
                                            Math.abs(backRightPower)))));

            frontLeft.setPower(frontLeftPower / max);
            backLeft.setPower(backLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backRight.setPower(backRightPower / max);

            if(gamepad1.b) {
                intaketest.setPower(1);
            }
            if(gamepad1.a){
                intaketest.setPower(-1);
            }
            if(gamepad1.x){
                intaketest.setPower(0);
            }
        }
    }
}
