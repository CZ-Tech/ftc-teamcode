package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;

public class LimelightCamera {
    private Robot robot;
    public Limelight3A camera;
    public LLResult result;

    public enum RESULT_ID{
        STATUS,
        CX,
        CY,
        W,
        H,
        ANGLE
    }

    public LimelightCamera(Robot robot){
        this.robot = robot;
        this.camera = robot.hardwareMap.get(Limelight3A.class, "limelight");

        camera.setPollRateHz(100);
        camera.pipelineSwitch(0);

        setTargetColor(Alliance.YELLOW);

//        camera.close();
    }

    public LimelightCamera start(){
        camera.start();
        return this;
    }

    public LimelightCamera stop(){
        camera.stop();
        return this;
    }

    public void setTargetColor(Alliance color){
        double[] inputs = {0, 2, 3, 4, 5, 6, 7, 8};

        switch (color){
            case RED: inputs[0] = 0;break;
            case BLUE: inputs[0] = 1;break;
            case YELLOW: inputs[0] = 2;break;
        }

        camera.updatePythonInputs(inputs);
    }

    public LimelightCamera update(){
        result = camera.getLatestResult();
        return this;
    }

    public LLResult getResult(){
        return result;
    }

    public double[] getPythonResults(){
        return result.getPythonOutput();
    }

    public LimelightCamera switchPipeline(int id){
        if (id < 0 || id > 9) return this;
        camera.pipelineSwitch(id);
        return this;
    }

    public double getData(RESULT_ID result_id){
        return getPythonResults()[result_id.ordinal()];
    }

    public boolean getStat(){
        return getData(RESULT_ID.STATUS) == 1;
    }

    public double getCx(){
        return getData(RESULT_ID.CX);
    }

    public double getCy(){
        return getData(RESULT_ID.CY);
    }

    public double getAngle(){
        return getData(RESULT_ID.ANGLE);
    }

    public double getWidth(){
        return getData(RESULT_ID.W);
    }

    public double getHeight(){
        return getData(RESULT_ID.H);
    }

    public double getTx(){
        return result.getTx();
    }

    public double getTy(){
        return result.getTy();
    }

    public double getTa(){
        return result.getTa();
    }
}
