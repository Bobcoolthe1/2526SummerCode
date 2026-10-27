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

@TeleOp(name = "Outtake Test", group = "JR")
public class outtakeTest extends LinearOpMode {

    DcMotorEx outtake;
    CRServoImplEx outtakeMiddle;

    private double motorPower = 0;
    private double servoPow = 0;

    @Override
    public void runOpMode() {

        outtake = hardwareMap.get(DcMotorEx.class, "O");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setVelocityPIDFCoefficients(0, 0, 0.0, 20);

        outtakeMiddle = hardwareMap.get(CRServoImplEx.class, "HS");
        outtakeMiddle.setDirection(CRServoImplEx.Direction.REVERSE);

        motorInitialize(outtake);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motorPower += 50;
                sleep(250);
            }
            if (gamepad1.dpad_down) {
                motorPower -= 50;
                sleep(250);
            }
            outtake.setVelocity(motorPower,AngleUnit.DEGREES);

            outtakeMiddle.setPower(1);
            motorInfo(outtake);
            telemetry.addData("getPower",outtake.getPower());
            telemetry.addData("PowerMotor",motorPower);
            telemetry.addData("PowerServo",servoPow);
            telemetry.addData("Speed",wheelSpeed(outtake));
            telemetry.addData("vel",outtake.getVelocity());
            telemetry.update();
        }
    }
    private void motorInitialize(DcMotorEx motor){
        motor.setMotorType(MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class));
        motor.setCurrentAlert(5000.0, CurrentUnit.MILLIAMPS);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // P=15 (Aggressive response), I=3 (Correction), D=0, F=11.7 (Calculated for REV HD Hex)
        motor.setVelocityPIDFCoefficients(12, 0.75, 0.0, 6.4);
        motor.setTargetPositionTolerance(2);
    }
    private void motorInfo(DcMotorEx motor){
        double a = motor.getVelocity(AngleUnit.DEGREES);
        double b = motor.getCurrentPosition();
        double c = motor.getCurrent(CurrentUnit.MILLIAMPS);
        telemetry.addData("Motor:", motor.getDeviceName());
        telemetry.addData("Velocity (deg/s)", a);
        telemetry.addData("CurrentPosition", b);
        telemetry.addData("Current (mA)", c);
    }
    private double wheelSpeed(DcMotorEx motor){
        double radius = 0.08;
        double pi = 3.1415;
        double degperSec = motor.getVelocity(AngleUnit.DEGREES);
        return (degperSec*(pi/180)) * radius;
    }

}
