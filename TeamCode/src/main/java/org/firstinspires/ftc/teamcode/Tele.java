package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ServoImpl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Tele")
public class Tele extends LinearOpMode {

    //Set Up
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx wheel, outtake, intake;
    private Servo servo;
    private ColorSensor FS,SS,TS;
    RobotCords robot;
    private Limelight3A limelight;
    //TODO: Write code for this ig
    private double Tag = 21;
    private GoBildaPinpointDriver OP;
    private VoltageSensor VS;

    //Wheel
    WheelState WheelMode = WheelState.IDLE;
    private int[] holes = new int[3];
    private int color;
    private int wheelTargetPos;
    private boolean colorMatch = true;

    //Outtake
    OuttakeState outtakeMode = OuttakeState.IDLE;
    private ElapsedTime OT = new ElapsedTime();
    private double shotTol = 300;
    private double servoSpeed = 150;

    //Constants
    private boolean Red = true;

    //Outtake
    public enum OuttakeState {
        IDLE,
        SPIN_UP,
        SHOT,
        UP,
        DOWN
    }
    private void outtakeLoop(){
        switch (outtakeMode){
            case IDLE:
                outtake.setPower(0);
                servo.setPosition(0);
                break;
            case SPIN_UP:
                outtake.setVelocity(wheelVel());
                if (Math.abs(outtake.getVelocity() - wheelVel()) < shotTol) {
                    if(WheelMode == WheelState.IDLE) {
                        outtakeMode = OuttakeState.SHOT;
                    }
                }
                break;
            case SHOT:
                servo.setPosition(1);
                OT.reset();
                outtakeMode = OuttakeState.UP;
                break;
            case UP:
                outtake.setVelocity(wheelVel());
                if(OT.milliseconds() > servoSpeed) {
                    servo.setPosition(0);
                    OT.reset();
                    outtakeMode = OuttakeState.DOWN;
                }
                break;
            case DOWN:
                outtake.setVelocity(wheelVel());
                if(OT.milliseconds() > servoSpeed) {
                    outtakeMode = OuttakeState.SPIN_UP;
                }
                break;
        }
    }
    private double wheelVel(){

        return 0;
    }
    public enum WheelState{
        IDLE,
        SLOT,
        MOVING,
        SHOT,
        INTAKE
    }
    private void wheelLoop(){
        switch(WheelMode){
            case IDLE:
                findHoles();
                wheel.setPower(0);
                colorMatch = true;
                break;
            case SHOT:
                findHoles();
                wheel.setPower(0);
                if(color == holes[0]){
                    wheelTargetPos = 200;
                }else if(color == holes[1]){
                    wheelTargetPos = 300;
                }else if(color == holes[2]){
                    wheelTargetPos = 400;
                }else{
                    colorMatch = false;
                }
                if(!colorMatch){
                    if(holes[0] == 1 || holes[0] == 2){
                        wheelTargetPos = 200;
                    }else if(holes[1] == 1 || holes[1] == 2){
                        wheelTargetPos = 300;
                    }else if(holes[2] == 1 || holes[2] == 2){
                        wheelTargetPos = 400;
                    }else{
                        wheelTargetPos = wheel.getCurrentPosition();
                    }
                }
                WheelMode = WheelState.MOVING;
                break;
            case INTAKE:
                if(0 == holes[0]){
                    wheelTargetPos = 200-180;
                }else if(0 == holes[1]){
                    wheelTargetPos = 300-180;
                }else if(0 == holes[2]){
                    wheelTargetPos = 400-180;
                }else{
                    wheelTargetPos = wheel.getCurrentPosition();
                }
                WheelMode = WheelState.MOVING;
                break;
            case MOVING:
                wheel.setTargetPosition(wheelTargetPos);
                wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wheel.setPower(0.5);

                double tol = 2;
                if(wheel.getCurrentPosition() - tol < wheelTargetPos &&
                        wheelTargetPos < wheel.getCurrentPosition() + tol){
                    WheelMode = WheelState.IDLE;
                }

                intake.setPower(0);
                break;
        }
    }
    private int getColor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        //Black == 0;
        //Purple == 1;
        //Green == 2;
        //Unknown == 3;
        if (r < 50 && g < 50 && b < 50) {
            return 0;
        } else if (r > g + 20 && b > g + 20) {
            return 1;
        } else if (g > r + 20 && g > b + 20) {
            return 2;
        } else {
            return 3;
        }
    }
    private void findHoles(){
        //has an outtake pos of 200
        holes[0] = getColor(FS);
        //has an outtake pos of 300
        holes[1] = getColor(SS);
        //has an outtake pos of 400
        holes[2] = getColor(TS);
    }
    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FL");
        backLeft = hardwareMap.get(DcMotorEx.class, "FL");
        backRight = hardwareMap.get(DcMotorEx.class, "FL");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        outtake = hardwareMap.get(DcMotorEx.class, "O");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        Increase P - If take to long to reach speed
        Decrease P - If it oscillates
        Increase D - If it overshoots
         */
        outtake.setVelocityPIDFCoefficients(20, 0, 5, 32767/13620);

        intake = hardwareMap.get(DcMotorEx.class, "I");
        wheel = hardwareMap.get(DcMotorEx.class, "W");

        servo = hardwareMap.get(Servo.class,"S");

        FS = hardwareMap.get(ColorSensor.class, "FS");
        SS = hardwareMap.get(ColorSensor.class, "SS");
        TS = hardwareMap.get(ColorSensor.class, "TS");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        GoBildaPinpointDriver OP = hardwareMap.get(GoBildaPinpointDriver.class, "OP");
        OP.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        OP.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        OP.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        OP.resetPosAndIMU();

        //DS = 0.5;

        //Robot Coordinates
        robot = new RobotCords(
                this,
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                wheel,
                outtake,
                intake,
                servo,
                OP,
                VS,
                limelight,
                Red
        );

        waitForStart();

        while (opModeIsActive()) {
            OP.update();
            if(gamepad2.x){
                OP.resetPosAndIMU();
            }

            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;

            double max = Math.max(1.0,
                    Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(backLeftPower),
                                    Math.max(Math.abs(frontRightPower),
                                            Math.abs(backRightPower)))));

            frontLeft.setPower(frontLeftPower / max);
            backLeft.setPower(backLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backRight.setPower(backRightPower / max);

            if(gamepad2.a){
                outtakeMode = OuttakeState.SPIN_UP;
            }
            if(gamepad2.b){
                outtakeMode = OuttakeState.IDLE;
            }
            outtakeLoop();

            if(gamepad2.left_bumper){
                color = 1;
                WheelMode = WheelState.SLOT;
            }
            if(gamepad2.right_bumper){
                color = 2;
                WheelMode = WheelState.SLOT;
            }
            wheelLoop();

            if(intake.getPower() > 0){
                WheelMode = WheelState.INTAKE;
            }
            if(gamepad1.a){
                intake.setPower(1);
            }

            if(gamepad2.dpad_down){
                Red = false;
            }
            if(gamepad2.dpad_up){
                Red = true;
            }

            Pose2D op = OP.getPosition();
            double X = -op.getY(DistanceUnit.INCH);
            double Y = op.getX(DistanceUnit.INCH);
            double Heading = op.getHeading(AngleUnit.DEGREES);
            telemetry.addData("OuttakeState",outtakeMode);
            telemetry.addData("IntakeState",WheelMode);
            telemetry.addData("Slot 1", getColor(FS));
            telemetry.addData("Slot 2", getColor(SS));
            telemetry.addData("Slot 3", getColor(TS));
            telemetry.addData("X",X);
            telemetry.addData("Y",Y);
            telemetry.addData("Heading",Heading);
            telemetry.addData("Red",Red);
            telemetry.update();
        }
    }
}