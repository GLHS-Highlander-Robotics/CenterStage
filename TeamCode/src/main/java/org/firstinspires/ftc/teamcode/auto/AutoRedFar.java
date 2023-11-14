package org.firstinspires.ftc.teamcode.auto;/* Copyright (c) 2017 FIRST. All rights reserved.
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


//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static org.firstinspires.ftc.teamcode.constants.AutoMods.*;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionNew;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.constants.AutoMods;

@Autonomous(name = "FarRedAuto")
public class AutoRedFar extends LinearOpMode{

    private SpikeDetectionNew spikeDetect;
    private VisionPortal portal;
    //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0),0));

    @Override
    public void runOpMode() throws InterruptedException {
        AutoMods.teamRed = true;
        LinearSlide slide = new LinearSlide(hardwareMap);

        spikeDetect = new SpikeDetectionNew(true);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(spikeDetect)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

//        Action moveArm = new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                slide.setArmPos(500, 200);
//                return false;
//            }
//        };
//        Action traj1 = drive.actionBuilder(drive.pose)
//                .splineTo(new Vector2d(48, 48), Math.PI / 2)
//                .stopAndAdd(moveArm)
//                .build();


        waitForStart();

        SpikeDetectionNew.Position position = spikeDetect.getPos();
        portal.close();

        switch (position) {
            case LEFT:
                break;
            case RIGHT:
                break;
            case CENTER:
                break;
            default:
                break;
        }

        while (opModeIsActive()) {
            if (isStopRequested()) {break;}
            slide.setRot(440);
            telemetry.addData("pos: ", slide.rotMotor.getCurrentPosition());
            telemetry.update();
        }
//        Actions.runBlocking (
//                new SequentialAction(
//                        traj1
//                )
//        );

    }
}
