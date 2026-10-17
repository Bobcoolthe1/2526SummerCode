package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "LimelightTest", group = "JR")
public class LimelightTest extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx outtake, intake;
    private CRServoImplEx intakeLeft, intakeRight, outtakeServo;
    private Limelight3A limelight;
    private VoltageSensor VS;
    private boolean red = true;
    private double tag = 0;
    private GoBildaPinpointDriver OP;

    // Tuning constants for ty
    static final double CAMERA_HEIGHT_IN = 8.0; // lens to floor
    static final double MOUNT_ANGLE_DEG = 25.0; // tilted down from horizontal
    static final double BALL_RADIUS_IN = 2.5; // measure a pollen ball

    static final int YELLOW = 0, RED = 1, BLUE = 2;

    private double ballResult = 4;

    Master master = new Master(
            this,
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            outtake,
            intake,
            intakeLeft,
            intakeRight,
            outtakeServo,
            OP,
            VS,
            limelight,
            red,
            tag
    );

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BL");
        backRight  = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Makes data refresh at 100x per second
        limelight.pipelineSwitch(0); // pollen-yellow pipeline
        limelight.start();
        waitForStart();

        OP = hardwareMap.get(GoBildaPinpointDriver.class, "OP");
        OP.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        OP.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        OP.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        OP.resetPosAndIMU();

        while (opModeIsActive()) {
            for (int i = 0; i < 2; i++){
                if(i == 0){
                    ballResult = 3;
                }
                sleep(50);
                limelight.pipelineSwitch(i);
                LLResult result = freshResult(i);
                result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    ballResult = i;
                    break;
                }
            }
            if(ballResult == 0){
                telemetry.addLine("Pollen - Yellow");
            }else if(ballResult == 1){
                telemetry.addLine("Nectar - Red");
            }else if(ballResult == 2){
                telemetry.addLine("Nectar - Blue");
            }else{
                telemetry.addLine("None");
            }
            telemetry.update();
        }
        limelight.stop();
    }
    LLResult freshResult(int slot) {
        LLResult r = limelight.getLatestResult();
        // Ignore the frame or two that still come from the previous pipeline
        if (r == null || !r.isValid() || r.getPipelineIndex() != slot) return null;
        return r;
    }

}