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

@Autonomous(name = "Auto Blue Close")
public class AutoFixedBlueClose extends LinearOpMode{

    private SpikeDetectionNew spikeDetect;
    private VisionPortal portal;
    FixedDrive drive;
    LinearSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoMods.teamRed = true;
        AutoMods.isFar = false;
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
            telemetry.update();
        }

        waitForStart();
        drive.imu.resetYaw();
        SpikeDetectionNew.Position position = spikeDetect.getPos();
        portal.close();

        switch (position) {
            case LEFT:
                //Move and place purple pixel
                slide.turnFloor();
                drive.rotateAndMoveInches(-90,37,-36,0.5,0.4);
                drive.rotateAndMoveInches(-90,0,4,0.2,0.4);
                slide.setArmPos(250,0);
                wait(0.25);
                slide.ungrabR();
                wait(0.25);
                slide.setArmPos(0, LinearSlide.LOW_ROT);

                slide.turnPlaceEx();
                wait(0.25);

                //Move and place yellow pixel on board
                drive.rotateAndMoveInches(-90,-20,-24,0.5,0.4);
                wait(0.6);
                slide.ungrabL();
                wait(0.25);

                //Position to go under gate
                drive.rotateAndMoveInches(-90,0,12,0.5,0.4);
                slide.setArmPos(0, LinearSlide.MEDIUM_ROT);
                drive.rotateAndMoveInches(-90,41,0,0.5,0.4);

                break;
            case RIGHT:
                //Move and place purple pixel
                slide.turnFloor();
                drive.rotateAndMoveInches(-90,35.5,-10,0.5,0.4);
                drive.rotateAndMoveInches(-90,0,4,0.2,0.4);
                slide.setArmPos(200,0);
                slide.ungrabR();
                wait(0.25);
                slide.setArmPos(0, LinearSlide.LOW_ROT);

                slide.turnPlaceEx();


                //Move and place yellow pixel on board
                drive.rotateAndMoveInches(-90,-3,-48,0.5,0.4);

                slide.ungrabL();
                wait(0.25);

                //Position to go under gate
                drive.rotateAndMoveInches(-90,0,14,0.5,0.4);
                slide.setAutoPos(0,LinearSlide.MEDIUM_ROT);
                drive.rotateAndMoveInches(-90,28.5,0,0.5,0.4);



                break;

            case CENTER:
                //Move and place purple pixel
                slide.turnFloor();
                drive.rotateAndMoveInches(0, 28.5, 0, 0.5, 0.2);
                slide.setArmPos(200,0);
    //                drive.rotateAndMoveInches(0,2,0, 0.2,0.2);
                slide.ungrabR();
                wait(0.25);
                slide.setArmPos(0,LinearSlide.LOW_ROT);
                slide.turnPlaceEx();


                //Place yellow pixel
                drive.rotateAndMoveInches(-90,-10.5,-48,0.5,0.4);

                slide.ungrabL();
                wait(0.25);

                //Position to get ready to go under the gate
                //strafe: -12
                drive.rotateAndMoveInches(-90,0,14,0.5,0.4);
                slide.setAutoPos(0, LinearSlide.MEDIUM_ROT);
                drive.rotateAndMoveInches(-90,38,0,0.5,0.4);
                break;

            default:
                break;
        }
        drive.rotateAndMoveInches(-90,0,0,0.2,0.3);
        drive.rotateAndMoveInches(-90,0,66,0.7,0.4);
        slide.setArmPos(200,0);
        drive.rotateAndMoveInches(-90,0,15,0.7,0.4);
        slide.turnFloor();
        drive.rotateAndMoveInches(-90,0,19,0.15,0.4);
        slide.grabR();
        slide.grabL();
        wait(0.5);
        slide.setArmPos(0, slide.MEDIUM_ROT);
        drive.rotateAndMoveInches(-90,0,-7,0.2,0.2);
        wait(0.5);
        drive.rotateAndMoveInches(-90,0,-80,0.7,0.4);
        drive.rotateAndMoveInches(-90,0,-7,0.5,0.4);

        slide.setArmPos(350,LinearSlide.LOW_ROT -10);

        //Go under the gate and rotate the arm (DEFINITELY CHANGE THIS CODE FOR DIFFERENT TEAMS)
//        drive.rotateAndMoveInches(90,0,0,0.2,0.7);
//        drive.rotateAndMoveInches(90,0,-66,1.0,0.4);
//        slide.setArmPos(120,0);
//        drive.rotateAndMoveInches(90,0,-20,1.0,0.4);
//
//        slide.turnFloor();
//        drive.rotateAndMoveInches(90,0,-20,0.15,0.4);




        switch (position){
            case LEFT:
                //Move to in front of the backboard (PROBABLY CHANGE THIS CODE FOR DIFFERENT TEAMS)
                slide.turnPlace();
                drive.rotateAndMoveInches(-90,-32,0,0.5,0.4);
                drive.rotateAndMoveInches(-90,0,-20,0.5,0.4);
                slide.ungrabR();
                slide.ungrabL();
                wait(0.5);
                break;

            case RIGHT:
                slide.turnPlace();
                drive.rotateAndMoveInches(-90,-32,0,0.5,0.4);
                drive.rotateAndMoveInches(-90,0,-15,0.5,0.4);
                wait(0.25);
                slide.ungrabR();
                slide.ungrabL();
                wait(0.5);
                break;
            case CENTER:
                slide.turnPlace();
                drive.rotateAndMoveInches(-90,-36,0,0.5,0.4);
                drive.rotateAndMoveInches(-90,0,-16,0.5,0.4);
                slide.ungrabR();
                slide.ungrabL();
                wait(0.5);

                break;
            default:
                break;
        }
        slide.setAutoPos(0,0);
        drive.rotateAndMoveInches(-5,0,8,0.5,0.5);


    }
    public void wait (double t) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time() < t) {idle();}
    }
}