package org.firstinspires.ftc.teamcode.common.vision;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionLimelight {
    private Robot robot;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    double[] results = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//(是否识别到)  ()  ()  () () () () () () () ()

    public VisionLimelight(Robot robot){
        this.robot = robot;
        robot.limelight.pipelineSwitch(6);
//        robot.limelight.close();
        robot.limelight.stop();
    }

    public double[] getAprilTagResults(int id){
//        robot.limelight.pipelineSwitch(id % 20);
//        robot.limelight.setPollRateHz(50);
//        robot.waitFor(250);
//        robot.limelight.start();

//        robot.limelight.start();

        LLResult result = robot.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        results[0] = 0;
        results[6] = 0;
        results[7] = 0;
        results[10] = getDistance(results[7]);
        if (!result.isValid()) return results;
        results[6] = result.getTx();
        results[7] = result.getTy();
        results[10] = getDistance(results[7]);


        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (!robot.opMode.opModeIsActive()) break;

            results[0] = 1;
            results[1] = fiducial.getFiducialId(); // 基准标记的ID号
            results[2] = fiducial.getTargetXDegrees();
            results[3] = fiducial.getTargetYDegrees();
            results[4] = fiducial.getRobotPoseTargetSpace().getPosition().x;
            results[5] = fiducial.getRobotPoseTargetSpace().getPosition().y;
        }
//        robot.limelight.pipelineSwitch(6);
//        robot.limelight.setPollRateHz(20);
//        robot.limelight.stop();
        return results;
    }

    public double getDistance(double ty){
        double x = ty;
        return -4.77201587855498e-7*x*x*x*x*x*x*x*x+0.0000398458912211183*x*x*x*x*x*x*x-0.0013963664728334623*x*x*x*x*x*x+0.02654445390468852*x*x*x*x*x-0.2947194100461383*x*x*x*x+1.9078738106287376*x*x*x-6.7042204973997945*x*x+10.275961018355073*x;
//        return 0.000001001681969590782*ty*ty*ty*ty*ty*ty*ty-0.00006389556712216817*ty*ty*ty*ty*ty*ty+0.0015378423415488055*ty*ty*ty*ty*ty-0.01596465147042008*ty*ty*ty*ty+0.034496247143979475*ty*ty*ty+0.6348813190858111*ty*ty-4.8889229144439605*ty+12.47317806342455;
//        return -154.055435312243*ty*ty*ty*ty*ty+1416.3900914504359*ty*ty*ty*ty-5123.048190446579*ty*ty*ty+9104.254750242788*ty*ty-7955.826533496639*ty+2752.8050001025854;
    }

    public int getTargetAprilTag(){
        robot.limelight.pipelineSwitch(2);
        robot.limelight.setPollRateHz(50);
        robot.limelight.start();

//        robot.limelight.start();

        LLResult result = robot.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (!result.isValid()) return 0;


        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (!robot.opMode.opModeIsActive()) break;

            if (fiducial.getFiducialId() >= 21 && fiducial.getFiducialId() <= 23){
//                robot.telemetry.addData("AprilTag", fiducial.getFiducialId());
                return fiducial.getFiducialId();
            }
        }
        return 0;
    }
}
