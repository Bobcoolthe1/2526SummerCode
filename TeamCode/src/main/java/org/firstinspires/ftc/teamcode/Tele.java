package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx outtake, intake;
    private CRServoImplEx outtakeLeft, outtakeRight, outtakeMiddle, conveyorServo;
    private Limelight3A limelight;
    private GoBildaPinpointDriver OP;
    private VoltageSensor VS;
    private boolean red = true;
    private boolean cycle = false;
    private double tag = 0;
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
        outtake.setVelocityPIDFCoefficients(20, 0, 5, 32767/13620);

        intake = hardwareMap.get(DcMotorEx.class, "I");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setVelocityPIDFCoefficients(20, 0, 5, 32767/13620);
        /*
        Increase P - If take to long to reach speed
        Decrease P - If it oscillates
        Increase D - If it overshoots
         */

        outtakeMiddle = hardwareMap.get(CRServoImplEx.class,"OM");
        outtakeMiddle.setDirection(CRServo.Direction.FORWARD);
        outtakeLeft = hardwareMap.get(CRServoImplEx.class,"OL");
        outtakeLeft.setDirection(CRServo.Direction.FORWARD);
        conveyorServo = hardwareMap.get(CRServoImplEx.class,"CS");
        conveyorServo.setDirection(CRServo.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        OP = hardwareMap.get(GoBildaPinpointDriver.class, "OP");
        OP.setOffsets(-88.6, -159.4, DistanceUnit.MM);
        OP.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        OP.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        OP.resetPosAndIMU();

        VS = hardwareMap.voltageSensor.iterator().next();

        Master master = new Master(
                this,
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                outtake,
                intake,
                outtakeMiddle,
                conveyorServo,
                outtakeLeft,
                outtakeRight,
                OP,
                VS,
                limelight,
                red,
                tag
        );

        waitForStart();

        while (opModeIsActive()) {

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

            //Movement
            if(gamepad1.dpad_down){
                master.cords(0,0,0);
            }
            if(gamepad1.x){
                OP.resetPosAndIMU();
            }

            //Cycle
            if(gamepad1.left_bumper){
                cycle = false;
            }
            if(gamepad1.right_bumper){
                cycle = true;
            }
            if(cycle){
                master.outtake();
                intake.setPower(1);
                outtakeMiddle.setPower(1);
                outtakeRight.setPower(1);
                outtakeLeft.setPower(1);
                conveyorServo.setPower(1);
            }

            //Outtake
            if(gamepad1.b){
                master.outtake();
            }

            //Intake
            if(gamepad1.a){
                intake.setPower(1);
                conveyorServo.setPower(1);
            }
            telemetry.update();
        }
    }
}