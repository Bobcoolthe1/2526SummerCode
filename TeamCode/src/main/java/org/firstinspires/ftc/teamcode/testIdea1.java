package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Deque;
public class testIdea1{
    private final LinearOpMode opmode;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight, outtake, intake;
    private final CRServoImplEx outtakeMiddle, conveyorServo, outtakeLeft, outtakeRight;
    public boolean red;
    private boolean state = false;
    public double tag;
    GoBildaPinpointDriver OP;
    private Limelight3A limelight;
    VoltageSensor batteryVoltage;
    //Movement Constants
    private final double positionTolerance = 0.5;
    private final double angleTolerance = 1;
    public testIdea1(LinearOpMode opmode,
                  DcMotorEx frontLeft,
                  DcMotorEx frontRight,
                  DcMotorEx backLeft,
                  DcMotorEx backRight,
                  DcMotorEx outtake,
                  DcMotorEx intake,
                  CRServoImplEx outtakeMiddle,
                  CRServoImplEx conveyorServo,
                  CRServoImplEx outtakeLeft,
                  CRServoImplEx outtakeRight,
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
        this.outtakeMiddle = outtakeMiddle;
        this.conveyorServo = conveyorServo;
        this.outtakeLeft = outtakeLeft;
        this.outtakeRight = outtakeRight;
        this.OP = OP;
        this.batteryVoltage = batteryVoltage;
        this.limelight = limelight;
        this.red = red;
        this.tag = tag;
    }
    private enum CordState {
        IDLE,
        MOVE,
        STOP
    }
    private CordState cordMode = CordState.IDLE;
    private double targetX;
    private double targetY;
    private double targetDegrees;
    public void startMoveToCoords(double x, double y, double degrees) {
        targetX = x;
        targetY = y;
        targetDegrees = degrees;
        cordMode = CordState.MOVE;
    }
    private void cordLoop() {
        switch (cordMode) {
            case IDLE:

                break;
            case MOVE:
                OP.update();
                Pose2D op = OP.getPosition();

                double heading = -op.getHeading(AngleUnit.RADIANS);
                double currentX = -op.getY(DistanceUnit.INCH);
                double currentY = op.getX(DistanceUnit.INCH);

                double deltaX = targetX - currentX;
                double deltaY = targetY - currentY;
                double distance = Math.hypot(deltaX, deltaY);
                double dirX = deltaX / distance;
                double dirY = deltaY / distance;

                double cosH = Math.cos(heading);
                double sinH = Math.sin(heading);

                double robotX = dirX * cosH - dirY * sinH;
                double robotY = dirX * sinH + dirY * cosH;

                double powerScale = 0.0404292*distance + 0.0297;

                if(distance >= 24){
                    powerScale = 1;
                }
                powerScale = Math.max(powerScale,0.1);

                robotX = robotX * powerScale;
                robotY = robotY * powerScale;

                //Turn
                double headingDegrees = -op.getHeading(AngleUnit.DEGREES);
                double error = targetDegrees - headingDegrees;

                double turnScale = 0.008*Math.abs(error) + 0.18;
                turnScale = Math.min(1, turnScale);

                double turnPower = (error < 0) ? -turnScale : turnScale;

                if(Math.abs(error) < angleTolerance){
                    turnPower = 0;
                }

                //Breakable
                if (distance < positionTolerance && Math.abs(error) < angleTolerance) {
                    cordMode = CordState.IDLE;
                }
                if(distance < positionTolerance){
                    cordMode = CordState.IDLE;
                }
                if (opmode.gamepad1.right_trigger > 0) {
                    stopMotors();
                    cordMode = CordState.IDLE;
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
                opmode.telemetry.update();
                break;
        }
    }
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}