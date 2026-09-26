package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Deque;
public class Master{
    private final LinearOpMode opmode;
    private final DcMotor frontLeft, frontRight, backLeft, backRight, IM, IM2, OL, OR;
    private final Servo OS, CS1, CS2;
    public boolean Red;
    GoBildaPinpointDriver OP;
    private Limelight3A limelight;
    VoltageSensor batteryVoltage;
    //Movement Constants
    private final double positionTolerance = 0.5;
    private final double angleTolerance = 1;
    public Master(LinearOpMode opmode,
                      DcMotor frontLeft,
                      DcMotor frontRight,
                      DcMotor backLeft,
                      DcMotor backRight,
                      DcMotor IM,
                      DcMotor IM2,
                      DcMotor OL,
                      DcMotor OR,
                      Servo OS,
                      GoBildaPinpointDriver OP,
                      VoltageSensor batteryVoltage,
                      Servo CS1,
                      Servo CS2,
                      Limelight3A limelight,
                      boolean Red
    ){
        this.opmode = opmode;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.IM = IM;
        this.IM2 = IM2;
        this.OL = OL;
        this.OR = OR;
        this.OS = OS;
        this.OP = OP;
        this.batteryVoltage = batteryVoltage;
        this.CS1 = CS1;
        this.CS2 = CS2;
        this.limelight = limelight;
        this.Red = Red;
    }
    public void cords(double X, double Y, double Degrees) {
        double powerScale;
        while (opmode.opModeIsActive()) {
            OP.update();
            Pose2D op = OP.getPosition();


            double heading = -op.getHeading(AngleUnit.RADIANS);
            double currentX = -op.getY(DistanceUnit.INCH);
            double currentY = op.getX(DistanceUnit.INCH);
            double Velx = -OP.getVelY(DistanceUnit.INCH);
            double Vely = OP.getVelX(DistanceUnit.INCH);
            double hypoVel = Math.hypot(Math.abs(Velx), Math.abs(Vely));

            double deltaX = X - currentX;
            double deltaY = Y - currentY;
            double distance = Math.hypot(deltaX, deltaY);
            double dirX = deltaX / distance;
            double dirY = deltaY / distance;

            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);

            double robotX = dirX * cosH - dirY * sinH;
            double robotY = dirX * sinH + dirY * cosH;


            /*

            powerScale = 0.02*(0.15*Math.pow(distance,2)+2*distance-1.5*hypoVel)+0.25;
            powerScale = Math.min(powerScale,1);

             */
            powerScale = 0.0404292*distance+0.0297;

            double minVelocity = 12.5;      // your threshold
            double kVelBoost = 0.02;       // tune this small

            double velError = minVelocity - hypoVel;
            if (velError > 0) {
                powerScale += kVelBoost * velError;
            }

            if(distance >= 24){
                powerScale = 1;
            }
            powerScale = Math.max(powerScale,0.1);

            robotX = robotX * powerScale;
            robotY = robotY * powerScale;

            if(distance < positionTolerance && hypoVel < 5){
                robotX = 0;
                robotY = 0;
            }

            // ---------- TURN CONTROL ----------
            double headingDegrees = -op.getHeading(AngleUnit.DEGREES);
            double error = Degrees - headingDegrees;

            double turnScale = 0.008 * Math.abs(error) + 0.18;
            turnScale = Math.min(1, turnScale);

            double turnPower = (error < 0) ? -turnScale : turnScale;

            if(Math.abs(error) < angleTolerance){
                turnPower = 0;
            }

            // ---------- MECANUM MIX ----------
            if (distance < positionTolerance && Math.abs(error) < angleTolerance) {
                break;
            }
            if (opmode.gamepad1.right_trigger > 0) {
                stopMotors();
                break;
            }

            double denom = Math.max(Math.abs(robotY) + Math.abs(robotX) + Math.abs(turnPower), 1);
            double fl = (robotY + robotX + turnPower) / denom;
            double bl = (robotY - robotX + turnPower) / denom;
            double fr = (robotY - robotX - turnPower) / denom;
            double br = (robotY + robotX - turnPower) / denom;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // Telemetry
            opmode.telemetry.addData("Distance", distance);
            opmode.telemetry.addData("hypoVel",hypoVel);
            opmode.telemetry.addData("Heading Error", error);
            opmode.telemetry.addData("RobotX",robotX);
            opmode.telemetry.addData("RobotY",robotY);
            opmode.telemetry.addData("TurnPower",turnPower);
            opmode.telemetry.addData("PowerScale",powerScale);
            opmode.telemetry.update();
            opmode.idle();
        }
        stopMotors();
    }
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    private static class VelSample {
        final double timeSec;
        final double velMps;

        VelSample(double timeSec, double velMps) {
            this.timeSec = timeSec;
            this.velMps = velMps;
        }
    }

    private final Deque<VelSample> velHistory = new ArrayDeque<>();
    private double accelMps2 = 0.0;
    // Call this every loop with current velocity (m/s)
    private void updateAcceleration(double Vel) {
        double nowSec = System.nanoTime() * 1e-9;

        velHistory.addLast(new VelSample(nowSec, Vel));

        // Keep only last 0.1 seconds of velocity samples
        double windowSec = 0.1;
        while (!velHistory.isEmpty() && (nowSec - velHistory.peekFirst().timeSec) > windowSec) {
            velHistory.removeFirst();
        }

        // Accel from oldest sample in 0.1s window to current sample
        if (velHistory.size() >= 2) {
            VelSample oldest = velHistory.peekFirst();
            double dt = nowSec - oldest.timeSec;
            if (dt > 1e-6) {
                accelMps2 = (Vel - oldest.velMps) / dt;
            }
        }
    }


}