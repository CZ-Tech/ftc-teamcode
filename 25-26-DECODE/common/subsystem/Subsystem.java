package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Subsystem {
    private final Robot robot;

    public final Intaker intaker;
    public final Thrower thrower;
    public final Shooter shooter;
    public final Belt belt;
    public final Door door;
    public final Classifier classifier;

    public Subsystem(Robot robot) {
        this.robot = robot;
        this.intaker = new Intaker(robot);
        this.thrower = new Thrower(robot);
        this.shooter = new Shooter(robot);
        this.belt = new Belt(robot);
        this.door = new Door(robot);
        this.classifier = new Classifier(robot);
    }
}
