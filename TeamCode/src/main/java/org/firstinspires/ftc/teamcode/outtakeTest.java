package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Outtake Test", group = "JR")
public class outtakeTest extends LinearOpMode {

    DcMotorEx outtake;
    ServoImplEx hoodServo;

    private double motorPower = 0.5;
    private double servoPos = 0.5;

    @Override
    public void runOpMode() {

        outtake = hardwareMap.get(DcMotorEx.class, "O");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(ServoImplEx.class, "HS");
        hoodServo.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motorPower += 0.01;
                sleep(250);
            }
            if (gamepad1.dpad_down) {
                motorPower -= 0.01;
                sleep(250);
            }

            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
            outtake.setPower(motorPower);

            if (gamepad1.dpad_right) {
                servoPos += 0.1;
                sleep(250);
            }
            if (gamepad1.dpad_left) {
                servoPos -= 0.1;
                sleep(250);
            }

            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            hoodServo.setPosition(servoPos);

            if(gamepad1.x){
                outtake.setDirection(DcMotorSimple.Direction.FORWARD);
            }


            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();
        }
    }
}
