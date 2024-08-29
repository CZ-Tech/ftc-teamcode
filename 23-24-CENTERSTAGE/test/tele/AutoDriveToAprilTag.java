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

package org.firstinspires.ftc.teamcode.test.tele;


import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@TeleOp(name = "Omni Drive To AprilTag", group = "Concept")
public class AutoDriveToAprilTag{
    public static void goToAprilTag(RobotHardware robot, double desiredDistance, int Tag_ID, double Delta_X) {
//        robot.init();
        robot.initAprilTagVision();
        robot.setManualExposure(4, 250);
        AprilTagDetection desiredTag = null;

        while (!robot.myOpMode.isStopRequested()) {
            desiredTag = robot.getAprilTag(Tag_ID);
            //            robot.driveStraight(0.2,1,0);
            if (desiredTag != null
                    && (Math.abs(desiredTag.ftcPose.range - desiredDistance) > 1
                    || Math.abs(desiredTag.ftcPose.bearing) > 3
//                    || Math.abs(desiredTag.ftcPose.yaw) > 12
            ))
            {
                robot.moveToAprilTag(desiredTag, desiredDistance,Delta_X);
//                telemetry.update();
            } else if (desiredTag != null && Math.abs(desiredTag.ftcPose.range - desiredDistance) < 1
                    && Math.abs(desiredTag.ftcPose.bearing) < 3
//                    && Math.abs(desiredTag.ftcPose.yaw) < 12
            ) {
                break;
            } else {
                robot.stopMotor();


//                miss++;
//                if(miss%2==1){
//                    robot.turnToHeading(0.3,robot.getHeading()+20);
//                    robot.sleep(1000);
//                }else{
//                    robot.turnToHeading(0.3,robot.getHeading()-40);
//                    robot.sleep(1000);
//                }

            }
        }
//        robot.driveStrafe(0.2, -2, robot.getHeading());
        robot.driveStraight(3, robot.getHeading(), 0.2);
    }
}