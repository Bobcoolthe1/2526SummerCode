package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "MecanumWheelDebugger", group = "Test")
public class MecDecoder extends LinearOpMode {

    // Hardware variables
    VoltageSensor battery = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private IMU imu = null;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.75;

            boolean buttonPressed = false;

            telemetry.addLine("X (Left)   ->  Front Left");
            telemetry.addLine("Y (Top)    ->  Front Right");
            telemetry.addLine("A (Bottom) ->  Back Left");
            telemetry.addLine("B (Right)  ->  Back Right");
            telemetry.addLine("--------------------------");

            if (gamepad1.x) {
                frontLeft.setPower(power);
                telemetry.addData("Active Motor", " Front Left (X)");
                buttonPressed = true;
            }else if (gamepad1.y) {
                frontRight.setPower(power);
                telemetry.addData("Active Motor", " Front Right (Y)");
                buttonPressed = true;
            }else if (gamepad1.a) {
                backLeft.setPower(power);
                telemetry.addData("Active Motor", " Back Left (A)");
                buttonPressed = true;
            }else if (gamepad1.b) {
                backRight.setPower(power);
                telemetry.addData("Active Motor", " Back Right (B)");
                buttonPressed = true;
            }else if (gamepad1.dpad_up) {
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
                telemetry.addData("Active Motor", " All Forward");
                buttonPressed = true;
            }else{
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            if (!buttonPressed) {
                telemetry.addLine("Active Motor:");
            }
            telemetry.addLine("--------------------------");
            telemetry.update();
        }
    }
}