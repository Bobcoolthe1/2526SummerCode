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
public class TeleV1 extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx outtake, intake;
    private CRServoImplEx outtakeLeft, outtakeRight, outtakeMiddle, conveyorServo;
    private Limelight3A limelight;
    private GoBildaPinpointDriver OP;
    private VoltageSensor VS;
    private boolean red = true;
    private boolean type = true;
    //type - true = pollen
    //type - false = nectar
    private double tag = 0;
    private boolean intakeOn = false;
    private boolean lastButton1 = false;
    private boolean outtakeOn = false;
    private boolean lastButton2 = false;
    private double outtakeVel = 0;
    private boolean outtakecolorOn = false;
    private boolean lastButton3 = false;

    @Override
    public void runOpMode() {

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
                type,
                red,
                tag
        );

        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake = hardwareMap.get(DcMotorEx.class, "O");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        master.motorInitialize(outtake);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotorEx.class, "I");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
        Increase P - If take to long to reach speed
        Decrease P - If it oscillates
        Increase D - If it overshoots
         */

        outtakeMiddle = hardwareMap.get(CRServoImplEx.class,"OM");
        outtakeMiddle.setDirection(CRServo.Direction.REVERSE);
        outtakeLeft = hardwareMap.get(CRServoImplEx.class,"OL");
        outtakeLeft.setDirection(CRServo.Direction.FORWARD);
        outtakeRight = hardwareMap.get(CRServoImplEx.class,"OR");
        outtakeRight.setDirection(CRServo.Direction.FORWARD);
        conveyorServo = hardwareMap.get(CRServoImplEx.class,"CS");
        conveyorServo.setDirection(CRServo.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        OP = hardwareMap.get(GoBildaPinpointDriver.class, "OP");
        OP.setOffsets(-88.6, -159.4, DistanceUnit.MM);
        OP.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        OP.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        OP.resetPosAndIMU();

        VS = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (opModeIsActive()) {
            OP.update();
            Pose2D op = OP.getPosition();

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
            if(gamepad1.dpad_left){
                master.cords(0,0,0);
            }
            if(gamepad1.dpad_right){
                master.cords(10,10,0);
            }
            if(gamepad1.x){
                OP.resetPosAndIMU();
            }

            //Outtake
            if(gamepad1.dpad_left){
                //Pollen
                outtake.setVelocity(1500,AngleUnit.DEGREES);
            }
            if(gamepad1.dpad_right){
                //Nectar
                outtake.setVelocity(1000,AngleUnit.DEGREES);
            }
            if (gamepad1.b && !lastButton2) {
                outtakeOn = !outtakeOn;
            }
            lastButton2 = gamepad2.b;
            if (outtakeOn) {
                outtakeMiddle.setPower(0.5);
            } else {
                outtakeMiddle.setPower(0);
            }

            //Intake
            if (gamepad1.a && !lastButton1) {
                intakeOn = !intakeOn;
            }
            lastButton1 = gamepad1.a;
            if (intakeOn) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            // conveyorServo
            if (intakeOn || outtakeOn) {
                conveyorServo.setPower(1);
            } else {
                conveyorServo.setPower(0);
            }

            //Intake Reverse
            if(gamepad1.left_trigger > 0.5){
                intake.setPower(-0.5);
            }
            if(gamepad1.right_trigger > 0.5){
                intake.setPower(0);
            }

            OP.update();
            telemetry.addData("setVel",outtakeVel);
            telemetry.addData("X", -op.getY(DistanceUnit.INCH));
            telemetry.addData("Y", -op.getX(DistanceUnit.INCH));
            master.motorInfo(outtake);
            telemetry.update();
        }
    }
}