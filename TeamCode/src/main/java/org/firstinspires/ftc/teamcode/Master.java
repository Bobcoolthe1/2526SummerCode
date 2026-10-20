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
public class Master{
    private final LinearOpMode opmode;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight, outtake, intake;
    private final CRServoImplEx outtakeMiddle, conveyorServo, outtakeLeft;
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
                      CRServoImplEx outtakeMiddle,
                      CRServoImplEx conveyorServo,
                      CRServoImplEx outtakeLeft,
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
    private void motorInitialize(DcMotorEx motor){
        motor.setMotorType(MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class));
        motor.setCurrentAlert(5000.0, CurrentUnit.MILLIAMPS);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // P=15 (Aggressive response), I=3 (Correction), D=0, F=11.7 (Calculated for REV HD Hex)
        motor.setVelocityPIDFCoefficients(12, 0.75, 0.0, 11.7);
        motor.setTargetPositionTolerance(2);
    }
    private void motorInfo(DcMotorEx motor){
        double a = motor.getVelocity(AngleUnit.DEGREES);
        double b = motor.getCurrentPosition();
        double c = motor.getCurrent(CurrentUnit.MILLIAMPS);
        opmode.telemetry.addData("Motor:", motor.getDeviceName());
        opmode.telemetry.addData("Velocity (deg/s)", a);
        opmode.telemetry.addData("CurrentPosition", b);
        opmode.telemetry.addData("Current (mA)", c);
    }
    private double wheelSpeed(DcMotorEx motor){
        double radius = 80;
        double pi = 3.1415;
        double degperSec = motor.getVelocity(AngleUnit.DEGREES);
        return (degperSec*(pi/180)) * radius;
    }
    FlyWheelState FlyWheelMode = FlyWheelState.IDLE;
    int ball = 4;
    ElapsedTime timer;
    double equation;
    private enum FlyWheelState{
        IDLE,
        SET,
        LAUNCH,
        WAIT
    }
    private void FlyWheelLoop() {
        switch (FlyWheelMode) {
            case IDLE:
                outtake.setVelocity(0, AngleUnit.DEGREES);
                outtakeMiddle.setPower(0);
                ball = ballColor();
                if(ball >= 0 && ball <=2) {
                    FlyWheelMode = FlyWheelState.SET;
                }
                break;
            case SET:
                double vel = wheelSpeed(outtake);
                if(ball == 0){
                    double pollenEquation = 1;
                    equation = pollenEquation;
                }else{
                    double nectarEquation = 2;
                    equation = nectarEquation;
                }
                outtake.setVelocity(equation, AngleUnit.DEGREES);

                double threshold = 10; // Increased threshold for stability
                double error = Math.abs(equation - vel);

                if (error < threshold) {
                    FlyWheelMode = FlyWheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                outtakeMiddle.setPower(1);
                timer.reset();
                FlyWheelMode = FlyWheelState.WAIT;
                break;
            case WAIT:
                if(timer.milliseconds() > 300){
                    outtakeMiddle.setPower(0);
                    FlyWheelMode = FlyWheelState.IDLE;
                }
                break;
        }
    }
    public void outtake(){
        motorInitialize(outtake);
        while(opmode.opModeIsActive()){

            FlyWheelLoop();

            motorInfo(outtake);
            opmode.telemetry.addData("Wheel Speed m/s",wheelSpeed(outtake));
            opmode.telemetry.update();
        }
    }
    public int ballColor(){
        int ballResult = 4;
        for (int i = 0; i <= 2; i++){
            if(i == 0){
                ballResult = 3;
            }
            limelight.pipelineSwitch(i);
            opmode.sleep(50);

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                ballResult = i;
                break;
            }
        }
        if(ballResult == 0){
            opmode.telemetry.addLine("Pollen - Yellow");
        }else if(ballResult == 1){
            opmode.telemetry.addLine("Nectar - Red");
        }else if(ballResult == 2){
            opmode.telemetry.addLine("Nectar - Blue");
        }else{
            opmode.telemetry.addLine("None");
        }
        return ballResult;
    }
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    private LLResult freshResult(int slot) {
        LLResult r = limelight.getLatestResult();
        // Ignore the frame or two that still come from the previous pipeline
        if (r == null || !r.isValid() || r.getPipelineIndex() != slot) return null;
        return r;
    }
    public void telemetry2(){
        if(opmode.gamepad2.dpad_up){
            //Cords

        }
        if(opmode.gamepad2.dpad_right){
            //Outtake
        }
        if(opmode.gamepad2.dpad_down){
            //Ballcolor
        }
        if(opmode.gamepad2.dpad_left){

        }
    }
}