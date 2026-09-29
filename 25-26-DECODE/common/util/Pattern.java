package org.firstinspires.ftc.teamcode.common.util;

public enum Pattern {
    GPP(21),
    PGP(22),
    PPG(23);

    private int id;
    Pattern(int id){
        this.id = id;
    }

    public int getId() {return id;}
}
