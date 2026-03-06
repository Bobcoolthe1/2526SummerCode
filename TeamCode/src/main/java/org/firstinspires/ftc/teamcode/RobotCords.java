package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;

public class RobotCords {
    private final LinearOpMode opmode;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final DcMotorEx wheel, outtake, intake;
    private final Servo servo;
    public boolean Red;
    GoBildaPinpointDriver OP;
    private Limelight3A limelight;
    VoltageSensor batteryVoltage;

    //Movement Constants
    private final double positionTolerance = 0.5;
    private final double angleTolerance = 1;
    private final double disForBooleans = 40;

    public RobotCords(LinearOpMode opmode,
                      DcMotorEx frontLeft,
                      DcMotorEx frontRight,
                      DcMotorEx backLeft,
                      DcMotorEx backRight,
                      DcMotorEx wheel,
                      DcMotorEx outtake,
                      DcMotorEx intake,
                      Servo servo,
                      GoBildaPinpointDriver OP,
                      VoltageSensor batteryVoltage,
                      Limelight3A limelight,
                      boolean Red
                      ){
        this.opmode = opmode;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.servo = servo;
        this.wheel = wheel;
        this.outtake = outtake;
        this.intake = intake;
        this.OP = OP;
        this.backRight = backRight;
        this.batteryVoltage = batteryVoltage;
        this.limelight = limelight;
        this.Red = Red;
    }
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void move(double X, double Y, double Degrees, boolean Set) {
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
            if(distance < disForBooleans){
                if(Set){
                    //IDK YET
                }
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
    public void outtake() {

    }
}