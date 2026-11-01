package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Outtake Test", group = "Test")
public class outtakeTest extends LinearOpMode {

    DcMotorEx outtake,conveyor;
    CRServoImplEx outtakeMiddle;

    private double motorPower = 1500;

    @Override
    public void runOpMode() {

        outtake = hardwareMap.get(DcMotorEx.class, "O");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMotorType(MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class));
        outtake.setCurrentAlert(5000.0, CurrentUnit.MILLIAMPS);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // P=15 (Aggressive response), I=3 (Correction), D=0, F=11.7 (Calculated for REV HD Hex)
        outtake.setVelocityPIDFCoefficients(12, 0.75, 0.0, 11.7);
        outtake.setTargetPositionTolerance(2);

        conveyor = hardwareMap.get(DcMotorEx.class, "CS");
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMiddle = hardwareMap.get(CRServoImplEx.class, "HS");
        outtakeMiddle.setDirection(CRServoImplEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motorPower += 10;
                sleep(250);
            }
            if (gamepad1.dpad_down) {
                motorPower -= 10;
                sleep(250);
            }
            outtake.setVelocity(motorPower,AngleUnit.DEGREES);

            outtakeMiddle.setPower(1);
            conveyor.setPower(1);

            telemetry.addData("setVel",motorPower);

            double a = outtake.getVelocity(AngleUnit.DEGREES);
            double b = outtake.getVelocity();
            telemetry.addData("Motor:", outtake.getDeviceName());
            telemetry.addData("Velocity (deg/s)", a);
            telemetry.addData("Velocity (tick/s)", b);

            telemetry.update();
        }
    }
}
