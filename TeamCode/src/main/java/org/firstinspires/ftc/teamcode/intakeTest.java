package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Intake Test", group = "JR")
public class intakeTest extends LinearOpMode {

    DcMotorEx intake;

    private double motorPower = 0.0;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotorEx.class, "I");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motorPower += 0.1;
                sleep(250);
            }
            if (gamepad1.dpad_down) {
                motorPower -= 0.1;
                sleep(250);
            }

            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
            intake.setPower(motorPower);

            telemetry.addData("Motor Power", motorPower);
            telemetry.update();
        }
    }
}
