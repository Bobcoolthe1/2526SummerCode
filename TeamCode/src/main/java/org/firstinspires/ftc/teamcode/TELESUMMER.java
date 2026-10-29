package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "DONT USE",group = "Bad")
public class TELESUMMER extends LinearOpMode {
    private GoBildaPinpointDriver OP;
    ElapsedTime timer = new ElapsedTime();
    private DcMotorEx LW, RW;
    private ServoImplEx Servo;
    private boolean Red = true;
    private double Balls = 0;
    boolean PowerScale = false;
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
        telemetry.addData("Motor:", motor.getDeviceName());
        telemetry.addData("Velocity (deg/s)", a);
        telemetry.addData("CurrentPosition", b);
        telemetry.addData("Current (mA)", c);
    }

    public enum FlyWheelState{
        IDLE,
        SET,
        LAUNCH,
        UP,
        DOWN
    }

    FlyWheelState FlyWheelMode = FlyWheelState.IDLE;
    double distanceEquationVel;

    private void FlyWheelLoop() {
        switch (FlyWheelMode) {
            case IDLE:
                LW.setVelocity(0, AngleUnit.DEGREES);
                RW.setVelocity(0, AngleUnit.DEGREES);
                Servo.setPosition(0);
                break;
            case SET:
                if(Balls != 0) {
                    distanceEquationVel = 3.05*(disGoal())+201;
                    double currentVel = (Math.abs(RW.getVelocity(AngleUnit.DEGREES)) + Math.abs(LW.getVelocity(AngleUnit.DEGREES))) / 2.0;
                    double target = distanceEquationVel;
                    LW.setVelocity(target, AngleUnit.DEGREES);
                    RW.setVelocity(target, AngleUnit.DEGREES);

                    double threshold = 10; // Increased threshold for stability
                    double error = Math.abs(target - currentVel);

                    if (error < threshold) { FlyWheelMode = FlyWheelState.LAUNCH; }

                    if(gamepad1.y){
                        FlyWheelMode = FlyWheelState.IDLE;
                    }

                    telemetry.addData("TargetDegPerSec", target);
                    telemetry.addData("Error", error);
                } else {
                    FlyWheelMode = FlyWheelState.IDLE;
                }
                break;
            case LAUNCH:
                Servo.setPosition(1);
                timer.reset();
                FlyWheelMode = FlyWheelState.UP;
                break;
            case UP:
                //TODO: Figure out the actual time for UP and DOWN
                if(timer.milliseconds() > 350){
                    Servo.setPosition(0);
                    timer.reset();
                    FlyWheelMode = FlyWheelState.DOWN;
                }
                break;
            case DOWN:
                if(timer.milliseconds() > 350){
                    Balls--;
                    FlyWheelMode = FlyWheelState.SET;
                }
                break;
        }
    }

    private double disGoal(){
        OP.update();
        Pose2D op = OP.getPosition();
        double currentX = -op.getY(DistanceUnit.INCH);
        double currentY = op.getX(DistanceUnit.INCH);

        double Reflect = Red ? 1 : -1;
        double GoalX = (72-6) * Reflect;
        double GoalY = 144-6;
        double deltaX = GoalX - currentX;
        double deltaY = GoalY - currentY;
        return Math.hypot(deltaX, deltaY);
    }

    private void startPos(double X, double Y){
        Pose2D AutonCords = new Pose2D(DistanceUnit.INCH, Y, -X, AngleUnit.DEGREES, 0);
        OP.setPosition(AutonCords);
    }

    @Override
    public void runOpMode() {
        LW = hardwareMap.get(DcMotorEx.class, "LW");
        RW = hardwareMap.get(DcMotorEx.class, "RW");

        motorInitialize(LW);
        motorInitialize(RW);
        RW.setDirection(DcMotor.Direction.REVERSE);

        Servo = hardwareMap.get(ServoImplEx.class, "Servo");
        Servo.scaleRange(0, 1);

        OP = hardwareMap.get(GoBildaPinpointDriver.class, "OP");
        OP.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        OP.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        OP.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        OP.resetPosAndIMU();

        startPos(14.25, 8.75);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x) startPos(14.25, 8.75);
            if(gamepad1.a) { Balls = 3; FlyWheelMode = FlyWheelState.SET; }
            if(gamepad1.b) { Balls = 1; FlyWheelMode = FlyWheelState.SET; }

            if (gamepad1.dpad_up && !PowerScale) {
                distanceEquationVel += 10;
            }if (gamepad1.dpad_down && !PowerScale) {
                distanceEquationVel = Math.max(0.0, distanceEquationVel - 10);
            }
            if(!gamepad2.dpad_right && !gamepad2.dpad_left){
                PowerScale = false;
            }

            if(!gamepad2.dpad_right && !gamepad2.dpad_left){
                PowerScale = false;
            }

            FlyWheelLoop();

            motorInfo(LW);
            telemetry.addData("State", FlyWheelMode);
            telemetry.addData("DISVEL",distanceEquationVel);
            telemetry.update();
        }
    }
}