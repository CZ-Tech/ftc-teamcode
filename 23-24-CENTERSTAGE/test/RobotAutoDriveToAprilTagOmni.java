/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Omni Drive To AprilTag", group = "Concept")
public class RobotAutoDriveToAprilTagOmni extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        robot.initAprilTagVision();
        robot.setManualExposure(3, 250);  // Use low exposure time to reduce motion blur
        waitForStart();
        AprilTagDetection desiredTag = null;
        int desiredDistance = 12;
        int Tag_ID = 1;
        while (!isStopRequested()) {
            desiredTag = robot.getAprilTag(Tag_ID);
            //            robot.driveStraight(0.2,1,0);
            if (desiredTag != null
                    && (Math.abs(desiredTag.ftcPose.range - desiredDistance) > 0.5
                    || Math.abs(desiredTag.ftcPose.bearing) > 3
                    || Math.abs(desiredTag.ftcPose.yaw) > 12)
            ) {
                robot.moveToAprilTag(desiredTag, desiredDistance,-1);
                telemetry.addData("status","moving");
            } else if (desiredTag != null
                    && (Math.abs(desiredTag.ftcPose.range - desiredDistance) <= 0.5
                    && Math.abs(desiredTag.ftcPose.bearing) <= 3
                    && Math.abs(desiredTag.ftcPose.yaw) <= 12)
            ) {
                telemetry.addData("status","done");
            } else {
                robot.stopMotor();
            }
            telemetry.update();
        }
//        robot.driveStrafe(0.2, -5, robot.getHeading());
//        robot.driveStraight(0.2, 3, robot.getHeading());

//        while (opModeIsActive()){
//            desiredTag = robot.getAprilTag(1);
//            if(desiredTag != null) {
//                robot.moveToAprilTag(desiredTag, 20);
//            }else{
//                robot.stopMotor();
//            }
//            telemetry.update();
//        }
//        robot.resetYaw();
//        robot.turnToHeading(0.5,0);
//        robot.stopMotor();
//        robot.resetYaw();
//        robot.turnToHeading(0.5,0);
//        robot.driveStraight(1,2,0);

        //万一没看到就转一转
        //                robot.turnToHeading(0.5,20);
        //                robot.resetYaw();
        //                sleep(2000);
        //                desiredTag = robot.getAprilTag(1);
        //                if(desiredTag == null){
        //                    robot.turnToHeading(0.5,-40);
        //                    robot.resetYaw();
        //                    sleep(2000);
        //                }

        //以下使用三角函数
        //            telemetry.addData("desiredTag.ftcPose.bearing:",desiredTag.ftcPose.bearing);
        //            telemetry.addData("desiredTag.ftcPose.range:",desiredTag.ftcPose.range);
        //            telemetry.update();

        //        telemetry.addData("desiredTag.ftcPose.range:",desiredTag.ftcPose.range);
        //        double yawMove = 0;
        //        if (Math.abs(yawError)<1){
        //            yawMove = 0;
        //        }
        //        else{
        //            yawMove = yawError*1.2;
        //            resetYaw();
        //            turnToHeading(0.2,yawMove);
        //        }
        //        double headingMove = 0;
        //        striveMove = Math.sin(desiredTag.ftcPose.bearing)*desiredTag.ftcPose.range;
        //        resetYaw();
        //        driveStrafe(0.3,striveMove,0);
        //


        //        robot.oneTimeMotor.setTargetPosition(robot.oneTimeMotor.getCurrentPosition()+100);
        //        robot.oneTimeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //        robot.setOneTimeMotorPower(0.5);
        //        sleep(2000);
        //        robot.oneTimeMotor.setTargetPosition(robot.oneTimeMotor.getCurrentPosition()-100);


    }
}