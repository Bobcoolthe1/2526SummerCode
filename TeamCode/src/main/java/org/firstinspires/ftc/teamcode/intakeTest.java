package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Intake Test", group = "JR")
public class intakeTest extends LinearOpMode {

    DcMotorEx intake;
    CRServoImplEx storgeIntake;

    private double power = 0.0;
    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotorEx.class, "I");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        storgeIntake = hardwareMap.get(CRServoImplEx.class,"SI");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                power += 0.1;
                sleep(250);
            }
            if (gamepad1.dpad_down) {
                power -= 0.1;
                sleep(250);
            }

            power = Math.max(-1.0, Math.min(1.0, power));
            intake.setPower(power);
            storgeIntake.setPower(power);

            telemetry.addData("Intake Power (servo/motor)", power);
            telemetry.update();
        }
    }
}
