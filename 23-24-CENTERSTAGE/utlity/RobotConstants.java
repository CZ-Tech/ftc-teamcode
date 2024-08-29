package org.firstinspires.ftc.teamcode.utlity;

public class RobotConstants {
    public enum IntakeArmPosition {
        BACK(690), // 1000
        GROUND(2410), // 2450
        BOARD(1780), // 1950
        AUTO_BOARD(1900), // 2010
        AUTO_EAT1(2335), //2385//2265
        AUTO_EAT2(2355),
        FRONT(270),
        TOP(1420),
        LIFTALITTLE(2220),
        ;
        private final int position;

        IntakeArmPosition(int position) {
            this.position = position;
        }

        public double getPosition() {
            return (position - 500) / 2000.0 ;
        }
    }
    public enum IntakeRollerPosition {
        IN(1000),
        SLOWIN(1200),
        OUT(2000),           
        STOP(1500)
        ;
        private final int position;

        IntakeRollerPosition(int position) {
            this.position = position;
        }

        public double getPosition() {
            return (position - 1000) / 1000.0 ;
        }
    }
    public enum IntakeFrontPosition {
        IN(2500),

        OUT(1500),
        ;
        private final int position;

        IntakeFrontPosition(int position) {
            this.position = position;
        }

        public double getPosition() {
            return (position - 500) / 2000.0 ;
        }
    }
    public enum IntakeBackPosition {
        IN(2500),
        OUT(1500),
        ;
        private final int position;

        IntakeBackPosition(int position) {
            this.position = position;
        }

        public double getPosition() {
            return (position - 500) / 2000.0 ;
        }
    }
}