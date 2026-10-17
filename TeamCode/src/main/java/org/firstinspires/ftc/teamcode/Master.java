package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight, outtake, intake;
    private final CRServoImplEx intakeLeft, intakeRight, outtakeServo;
    public boolean red;
    public double tag;
    GoBildaPinpointDriver OP;
    private Limelight3A limelight;
    VoltageSensor batteryVoltage;
    //Movement Constants
    private final double positionTolerance = 0.5;
    private final double angleTolerance = 1;
    public Master(LinearOpMode opmode,
                      DcMotorEx frontLeft,
                      DcMotorEx frontRight,
                      DcMotorEx backLeft,
                      DcMotorEx backRight,
                      DcMotorEx outtake,
                      DcMotorEx intake,
                      CRServoImplEx intakeLeft,
                      CRServoImplEx intakeRight,
                      CRServoImplEx outtakeServo,
                      GoBildaPinpointDriver OP,
                      VoltageSensor batteryVoltage,
                      Limelight3A limelight,
                      boolean red,
                      double tag
    ){
        this.opmode = opmode;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.outtake = outtake;
        this.intake = intake;
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
        this.outtakeServo = outtakeServo;
        this.OP = OP;
        this.batteryVoltage = batteryVoltage;
        this.limelight = limelight;
        this.red = red;
        this.tag = tag;
    }
    public void cords(double X, double Y, double Degrees) {
        double powerScale;
        while (opmode.opModeIsActive()) {
            OP.update();
            Pose2D op = OP.getPosition();

            double heading = -op.getHeading(AngleUnit.RADIANS);
            double currentX = -op.getY(DistanceUnit.INCH);
            double currentY = op.getX(DistanceUnit.INCH);

            double deltaX = X - currentX;
            double deltaY = Y - currentY;
            double distance = Math.hypot(deltaX, deltaY);
            double dirX = deltaX / distance;
            double dirY = deltaY / distance;

            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);

            double robotX = dirX * cosH - dirY * sinH;
            double robotY = dirX * sinH + dirY * cosH;

            powerScale = 0.0404292*distance+0.0297;

            if(distance >= 24){
                powerScale = 1;
            }
            powerScale = Math.max(powerScale,0.1);

            robotX = robotX * powerScale;
            robotY = robotY * powerScale;

            if(distance < positionTolerance){
                robotX = 0;
                robotY = 0;
            }

            //Turn
            double headingDegrees = -op.getHeading(AngleUnit.DEGREES);
            double error = Degrees - headingDegrees;

            double turnScale = 0.008 * Math.abs(error) + 0.18;
            turnScale = Math.min(1, turnScale);

            double turnPower = (error < 0) ? -turnScale : turnScale;

            if(Math.abs(error) < angleTolerance){
                turnPower = 0;
            }

            //Breakable
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
}