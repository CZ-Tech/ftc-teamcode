package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class Ingrabber {
    private final Robot robot;
    public CRServoImplEx rightingrabber;
    public ServoImplEx rightturning;

    public CRServoImplEx leftingrabber;
    public ServoImplEx leftturning;

    public static double RIGHT_EAT_POS = 2250; //TODO modify
    public static double RIGHT_PUT_POS = 1100; //TODO modify
    public static double RIGHT_AUTO_INIT_POS = 1100;

    public static double LEFT_EAT_POS = 700; //TODO modify
    public static double LEFT_PUT_POS = 2100; //TODO modify
    public static double LEFT_AUTO_INIT_POS = 2100;

    public static double BackTime = 700.0;


    public Ingrabber(Robot robot){
        this.robot = robot;
        rightingrabber = robot.hardwareMap.get(CRServoImplEx.class, "rightingrabber");//eh 3
        rightturning = robot.hardwareMap.get(ServoImplEx.class, "rightturning");//eh 2
        rightturning.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftingrabber = robot.hardwareMap.get(CRServoImplEx.class, "leftingrabber");//ch 3
        leftturning = robot.hardwareMap.get(ServoImplEx.class, "leftturning");//ch 2
        leftturning.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public Ingrabber left_takeIn(){
        leftingrabber.setPower(-1.0);
        return this;
    }

    public Ingrabber left_takeOut(){
        leftingrabber.setPower(1.0);
        return this;
    }

    public Ingrabber left_stop(){
        leftingrabber.setPower(0.0);
        return this;
    }

    public Ingrabber left_turnTo(double movecounts){
        leftturning.setPosition((movecounts - 500.0) / 2000.0);
        return this;
    }

    public Ingrabber left_EatPos(){
        left_turnTo(LEFT_EAT_POS);
        return this;
    }


    //让ingrabber吃sample
    public Ingrabber left_Eat(){//TODO待测试
        left_EatPos();
        left_takeIn();
        return this;
    }

    //让ingrabber吐出sample
    public  Ingrabber left_Put(){//TODO待测试
        robot.syncRun(()->{
                    left_PutPos();},
        ()->{
            robot.waitFor(BackTime);
//            left_takeOut();
        }) ;
        return this;

    }

    public Ingrabber left_PutPos(){
        left_turnTo(LEFT_PUT_POS);
        return this;
    }

    public Ingrabber left_auto_init(){
        left_turnTo(LEFT_AUTO_INIT_POS);
        return this;

    }






    public Ingrabber right_takeIn(){
        rightingrabber.setPower(1.0);
        return this;
    }

    public Ingrabber right_takeOut(){
        rightingrabber.setPower(-1.0);
        return this;
    }

    public Ingrabber right_stop(){
        rightingrabber.setPower(0.0);
        return this;
    }

    public Ingrabber right_turnTo(double movecounts){
        rightturning.setPosition((movecounts - 500.0) / 2000.0);
        return this;
    }

    public Ingrabber right_EatPos(){
        right_turnTo(RIGHT_EAT_POS);
        return this;
    }


    //让ingrabber吃sample
    public Ingrabber right_Eat(){//TODO待测试
        right_EatPos();
        right_takeIn();
        return this;
    }

    //让ingrabber吐出sample
    public  Ingrabber right_Put(){//TODO待测试
        robot.syncRun(()->{
                    right_PutPos();},
                ()->{
                    robot.waitFor(BackTime);
//                    right_takeOut();
        }) ;
        return this;

    }

    public Ingrabber right_PutPos(){
        right_turnTo(RIGHT_PUT_POS);
        return this;
    }

    public Ingrabber right_auto_init(){
        return this.right_turnTo(RIGHT_AUTO_INIT_POS);
    }

}