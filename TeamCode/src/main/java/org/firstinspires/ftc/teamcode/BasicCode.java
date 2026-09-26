package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BasicCode", group = "JR")
public class BasicCode extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {

        // --- Hardware Map ---
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft   = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");

        // --- Motor Directions ---
        // Reverse LEFT side for most FTC drivetrains
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Wait for Start ---
        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad Axes ---
            double drive  = -gamepad1.left_stick_y;   // forward/back
            double strafe =  gamepad1.left_stick_x;   // left/right
            double turn   =  gamepad1.right_stick_x;  // rotation

            // --- Mecanum Math ---
            double frontLeftPower  = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower   = drive - strafe + turn;
            double backRightPower  = drive + strafe - turn;

            // --- Normalize (so no motor exceeds 1.0) ---
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // --- Send Power ---
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

        }
    }
}
