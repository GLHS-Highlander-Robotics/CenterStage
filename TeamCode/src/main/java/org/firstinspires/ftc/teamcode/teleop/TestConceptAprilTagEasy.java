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

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ConceptAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name = "Concept: AprilTag Easy", group = "Concept")
public class TestConceptAprilTagEasy extends LinearOpMode {



    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0),0));
        ConceptAprilTag aptag = new ConceptAprilTag();
        aptag.initAprilTag(this);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                aptag.telemetryAprilTag();
                if (aptag.tagDetects().size() >= 1) {
                    for (AprilTagDetection detection : aptag.tagDetects()) {
                        if (detection.id == 8) {
                            if (detection.ftcPose.x > 0.1 || detection.ftcPose.x < -0.1) {
                                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.1 * detection.ftcPose.x * 5, 0), 0));
                            } else {
                                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                            }
                            } else {
                                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                            }
                        }
                    }
                else {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                }

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    aptag.apvisionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    aptag.apvisionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        aptag.apvisionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */


    /**
     * Add telemetry about AprilTag detections.
     */

}   // end class
