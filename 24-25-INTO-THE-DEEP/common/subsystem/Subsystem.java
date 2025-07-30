package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Subsystem {

    public Intaker intaker;
//    public final Thrower thrower;
//    public SlamDunker slamDunker;
    public Arm arm;
    public final Robot robot;
    public Stretcher stretcher;
    public Grabber grabber;
    public Ingrabber ingrabber;
    public Thrower thrower;
    public Claw claw;
    public NewClaw newclaw;


    public Subsystem(Robot robot) {
        this.robot = robot;
//        this.intaker = new Intaker(robot);
//        this.thrower = new Thrower(robot);
//        this.slamDunker = new SlamDunker(robot);
        this.arm = new Arm(robot);
        this.stretcher = new Stretcher(robot);
        this.grabber = new Grabber(robot);
        this.ingrabber = new Ingrabber(robot);
//        this.thrower = new Thrower(robot);
        this.claw = new Claw(robot);
        this.newclaw = new NewClaw(robot);
    }
}
