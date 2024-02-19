/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.AutoMods;
import org.firstinspires.ftc.teamcode.subsystem.drive.FixedDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionNew;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Auto Red Far")
public class AutoFixedRedFar extends LinearOpMode{

    private SpikeDetectionNew spikeDetect;
    private VisionPortal portal;
    FixedDrive drive;
    LinearSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoMods.teamRed = true;
        AutoMods.isFar = true;
        slide = new LinearSlide(hardwareMap);
        drive = new FixedDrive(hardwareMap, this);
        drive.botHeading = 0;


        spikeDetect = new SpikeDetectionNew();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(spikeDetect)
                .build();

        slide.turnPlaceAuto();
        slide.grabAll();
        slide.turnRot(slide.droneServo, 1);

        while(opModeInInit()){
            telemetry.addData("Right Claw","Purple Pixel");
            telemetry.addData("Left Claw","Yellow Pixel");
            telemetry.addData("Block Location", spikeDetect.getPos());
            telemetry.addData("Slide Alignment", "Left Side to Hole");
            telemetry.update();
        }

        waitForStart();
        drive.imu.resetYaw();
        SpikeDetectionNew.Position position = spikeDetect.getPos();
        portal.close();

        switch (position) {
            case RIGHT:
                //Move and place purple pixel
                slide.turnFloor();
                drive.rotateAndMoveInches(-90,36,-16,0.5,0.4);
                drive.rotateAndMoveInches(-90,0,5,0.2,0.2);
                slide.setArmPos(450,0);
                wait(0.25);
                slide.ungrabR();
                wait(0.25);
                slide.setArmPos(35,0);
                slide.turnRot(slide.leftRot, LinearSlide.LPLACE + 0.29);
                wait(0.25);

                //Go to white stack
                drive.rotateAndMoveInches(90, -4, -22, 0.5,0.5);
                wait(0.25);
                drive.rotateAndMoveInches(90, 0, -7.5, 0.2,0.2);
                slide.grabR();
                wait(0.25);
                drive.rotateAndMoveInches(90,0,4,0.2,0.2);

                //Position to get ready to go under the gate
                slide.turnRot(slide.rightRot, LinearSlide.RPLACE + 0.29);
                slide.setArmPos(0,0);
                drive.rotateAndMoveInches(90,0,14,0.5,0.4);
                drive.rotateAndMoveInches(90,-21.25,8,0.5,0.4);

                //End Position (90, 5, -5.5)
                break;
            case LEFT:
                //Move and place purple pixel
                slide.turnFloor();
                slide.setArmPos(250,0);
                drive.rotateAndMoveInches(90,35,6,0.5,0.4);
                wait(0.25);
                slide.ungrabR();
                wait(0.25);
                slide.setArmPos(35,0);
                slide.turnRot(slide.leftRot, LinearSlide.LPLACE + 0.29);
                wait(0.25);

                //Go to white stack
                drive.rotateAndMoveInches(90, -10, 0, 0.5,0.5);
                drive.rotateAndMoveInches(90,0,-20,0.5,0.25);
                wait(0.25);
                drive.rotateAndMoveInches(90, 0, -7.5, 0.2,0.2);
                slide.grabR();
                wait(0.25);
                drive.rotateAndMoveInches(90,0,4,0.2,0.2);

                //Position to get ready to go under the gate
                slide.turnRot(slide.rightRot, LinearSlide.RPLACE + 0.29);
                slide.setArmPos(0,0);
                drive.rotateAndMoveInches(90,0,16,0.5,0.4);
                drive.rotateAndMoveInches(90,-20,6,0.5,0.4);

                //End Position (90, 5, -5.5)
                break;
            case CENTER:
                //Move and place purple pixel
                slide.turnFloor();
                slide.setArmPos(200,0);
                drive.rotateAndMoveInches(0, 30, 0, 0.5, 0.2);
//                drive.rotateAndMoveInches(0,2,0, 0.2,0.2);
                slide.ungrabR();
                slide.setArmPos(35,0);
                slide.turnRot(slide.leftRot, LinearSlide.LPLACE + 0.29);
                wait(0.25);

                //Go to white stack
                drive.rotateAndMoveInches(0,-3.5,0,0.5,0.5);
                drive.rotateAndMoveInches(90, 0, -24,0.5,0.5);
                wait(0.25);
                drive.rotateAndMoveInches(90,0,-7.5,0.2,0.2);
                wait(0.25);
                slide.grabR();
                wait(0.25);
                drive.rotateAndMoveInches(90,0,4,0.2,0.2);
                //Stack Position: (90, 27, -27.5)

                //Position to get ready to go under the gate
                slide.turnRot(slide.rightRot, LinearSlide.RPLACE + 0.29);
                slide.setArmPos(0,0);
                drive.rotateAndMoveInches(90,0,16,0.5,0.4);
                drive.rotateAndMoveInches(90,-21.5,6,0.5,0.4);

                //End Position (90, 5, -5.5)
                break;
            default:
                break;
        }
        //Go under the gate and rotate the arm (DEFINITELY CHANGE THIS CODE FOR DIFFERENT TEAMS)
        drive.rotateAndMoveInches(90, 0,66,0.5,0.2);
        slide.setArmPos(150, LinearSlide.LOW_ROT - 20);
        slide.turnPlaceEx();
        wait(0.5);



        switch (position){
            case LEFT:
                //Move to in front of the backboard (PROBABLY CHANGE THIS CODE FOR DIFFERENT TEAMS)
                drive.rotateAndMoveInches(90, 34, 0, 0.5, 0.2);

                //Move into the backboard and release
                drive.rotateAndMoveInches(90,0,20,0.25,0.2);
                slide.ungrabL();
                wait(0.25);
                drive.rotateAndMoveInches(90,-20,-12,0.25,0.25);
                wait(0.25);
                drive.rotateAndMoveInches(90,0,16,0.25,0.25);
                slide.ungrabR();
                wait(0.5);
                slide.setArmPos(0,0);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90,0,-4,0.25,0.2);
                drive.rotateAndMoveInches(90,-20, 0,0.25,0.25);
                slide.setAutoPos(0,0);
                break;

            case RIGHT:
                //Move to in front of the backboard (PROBABLY CHANGE THIS CODE FOR DIFFERENT TEAMS)
                drive.rotateAndMoveInches(90, 22, 0, 0.5, 0.2);

                //Move into the backboard and release
                drive.rotateAndMoveInches(90,0,20,0.25,0.2);
                slide.ungrabAll();
                wait(0.5);
                slide.setArmPos(0,0);
                slide.turnPlaceEx();
                //Leave the backboard to allow alliance to place
                drive.rotateAndMoveInches(90,0,-6,0.25,0.2);
                drive.rotateAndMoveInches(90,-21.75, 0,0.25,0.25);
                slide.setAutoPos(0,0);
                break;
            case CENTER:
                //Move to in front of the backboard (PROBABLY CHANGE THIS CODE FOR DIFFERENT TEAMS)
                drive.rotateAndMoveInches(90, 29.5, 0, 0.5, 0.2);

                //Move into the backboard and release
                drive.rotateAndMoveInches(90,0,20,0.25,0.2);
                slide.ungrabL();
                slide.setArmPos(300,LinearSlide.LOW_ROT);
                wait(0.25);
                drive.rotateAndMoveInches(90,-6,0,0.25,0.25);
                wait(0.25);
                slide.ungrabR();
                wait(0.5);
                slide.setArmPos(0,0);
                slide.turnPlaceEx();
                drive.rotateAndMoveInches(90,0,-4,0.25,0.2);
                drive.rotateAndMoveInches(90,-24, 0,0.25,0.25);
                slide.setAutoPos(0,0);
                break;
            default:
        }
        drive.rotateAndMoveInches(15,0,0,0.25,1);
    }
    public void wait (double t) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time() < t) {idle();}
    }
}
