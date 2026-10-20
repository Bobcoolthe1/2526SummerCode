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
    private Limelight3A limelight;
    private double ballResult = 4;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Makes data refresh at 100x per second
        limelight.pipelineSwitch(0); // pollen-yellow pipeline
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i <= 2; i++){
                if(i == 0){
                    ballResult = 3;
                }
                limelight.pipelineSwitch(i);
                sleep(50);

                LLResult result = limelight.getLatestResult();
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