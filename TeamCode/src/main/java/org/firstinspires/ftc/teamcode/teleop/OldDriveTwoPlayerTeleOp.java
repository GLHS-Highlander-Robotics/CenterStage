package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.HIGH_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.INCREMENT_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.INCREMENT_STEPS_SLIDE;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.LOW_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.LOW_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MAX_POWER_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MAX_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MEDIUM_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MEDIUM_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MIN_ROT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.AutoMods;
import org.firstinspires.ftc.teamcode.subsystem.drive.OldDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;

@Config
@TeleOp (name = "OGTwoPlayerTeleop")
public class OldDriveTwoPlayerTeleOp extends LinearOpMode {
    public static double HIGH_POWER = 0.65;
    public static double LOW_POWER = 0.35;
    public static double DEAD_ZONE_P1 = 0.05;
    public static double DEAD_ZONE_P2 = 0.05;

    OldDrive drive;
    LinearSlide slide;

    double limiter = HIGH_POWER;
    boolean fieldCentric = true;
    double botHeading;

    double forward = 0;
    double strafe = 0;
    double rotate = 0;
    double fieldForward = 0;
    double fieldStrafe = 0;

    int armMotorSteps = 0;
    int rotMotorSteps = 0;

    boolean leftStickPressed = false;
    boolean rightStickPressed = false;
    boolean leftGrabbed = false;
    boolean rightGrabbed = false;
    boolean detectedR = false;
    boolean detectedL = false;
    boolean detectedRot = false;
    boolean detectedRotTrig = false;
    boolean rotTrigged = false;
    boolean detectedResetTrig = false;
    boolean resetTrigged = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry multTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new OldDrive(hardwareMap, this);
        slide = new LinearSlide(hardwareMap);

        drive.setModes(DcMotorEx.RunMode.RUN_USING_ENCODER);
        drive.imu.resetYaw();
        slide.ungrabL();
        slide.ungrabR();
        slide.turnFloor();
        slide.turnRot(slide.droneServo, 1);
        slide.place = false;
        while (opModeInInit()) {
            updateTeleOpTelemetry();
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            updateDriveByGamepad();
            updateSlideByGamepad();
            updateTeleOpTelemetry();
            telemetry.update();
        }
    }

    public void updateSlideByGamepad() {
        slide.update();
        // Set rotation steps to predefined height with p2 buttons, preset pickup command
        if (gamepad2.a) {
            rotMotorSteps = MIN_ROT;
            armMotorSteps = MIN_HEIGHT;
            slide.turnFloor();
            slide.place = false;
        } else if (gamepad2.b) {
            rotMotorSteps = LOW_ROT;
            armMotorSteps = MIN_HEIGHT;
            slide.turnPlaceEx();
            slide.place=true;
        } else if (gamepad2.x) {
            rotMotorSteps = MEDIUM_ROT;
            armMotorSteps = MIN_HEIGHT;
            slide.turnPlaceEx();
            slide.place=true;
        } else if (gamepad2.y) {
            rotMotorSteps =HIGH_ROT;
            armMotorSteps = MIN_HEIGHT;
        }

        // Increase arm and rotation steps by increments using p2 sticks
        if (-gamepad2.left_stick_y > DEAD_ZONE_P2) {
            armMotorSteps += INCREMENT_STEPS_SLIDE;
            leftStickPressed = true;
        } else if (-gamepad2.left_stick_y < -DEAD_ZONE_P2) {
            armMotorSteps -= INCREMENT_STEPS_SLIDE;
            leftStickPressed = true;
        } else if (leftStickPressed) {
            leftStickPressed = false;
        }

        // Raise Slide to Presets
        if (gamepad2.left_bumper) {
            armMotorSteps = LOW_HEIGHT;
        } else if (gamepad2.right_bumper) {
            armMotorSteps = MEDIUM_HEIGHT;
        }

        //incremental rotation with p2 dpad

        if (gamepad2.dpad_up) {
            rotMotorSteps += INCREMENT_ROT;
            detectedRot = true;
        } else if (gamepad2.dpad_down) {
            rotMotorSteps -= INCREMENT_ROT;
            detectedRot = true;
        } else if (detectedRot) {
            detectedRot = false;
        }
        armMotorSteps = Range.clip(armMotorSteps, MIN_HEIGHT, MAX_HEIGHT);
        rotMotorSteps = Range.clip(rotMotorSteps, MIN_ROT - 100, MAX_HEIGHT);
        slide.setArmPos(armMotorSteps, rotMotorSteps);

        //Grab claw with p1 bumpers

        if (!detectedR) {
            if (gamepad1.right_trigger > 0.5 && rightGrabbed) {
                slide.ungrabR();
                rightGrabbed = false;
                detectedR = true;
            } else if (gamepad1.right_trigger > 0.5 && !rightGrabbed) {
                slide.grabR();
                rightGrabbed = true;
                detectedR = true;
            }
        } else {
            if (gamepad1.right_trigger < 0.5) {
                detectedR = false;
            }
        }
        if (!detectedL) {
            if (gamepad1.left_trigger > 0.5 && leftGrabbed) {
                slide.ungrabL();
                leftGrabbed = false;
                detectedL = true;
            } else if (gamepad1.left_trigger > 0.5 && !leftGrabbed) {
                slide.grabL();
                leftGrabbed = true;
                detectedL = true;
            }
        } else {
            if (gamepad1.left_trigger < 0.5) {
                detectedL = false;
            }
        }
        // rotation trigger
        if (!detectedRotTrig) {
            if (gamepad2.right_trigger > 0.5 && rotTrigged) {
                slide.place = true;
                detectedRotTrig = true;
                rotTrigged = false;

            } else if (gamepad2.right_trigger > 0.5 && !rotTrigged) {
                slide.place = false;
                detectedRotTrig = true;
                rotTrigged = true;

            }
        } else {
            if (gamepad2.right_trigger < 0.5)
                detectedRotTrig = false;
        }

        if (gamepad2.left_trigger > 0.5) {
            slide.turnRot(slide.droneServo, 0);
        }


//        if (!detectedResetTrig) {
//            if (gamepad2.left_trigger > 0.5 && resetTrigged) {
//                slide.rotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                detectedResetTrig = true;
//                resetTrigged = false;
//
//            } else if (gamepad2.left_trigger > 0.5 && !resetTrigged) {
//                slide.rotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                slide.rotMotor.setPower(MAX_POWER_ROT);
//                detectedResetTrig = true;
//                resetTrigged = true;
//
//            }
//        } else {
//            if (gamepad2.left_trigger < 0.5) {
//                detectedResetTrig = false;
//            }
//
//
//        }
    }






    public void updateDriveByGamepad() {
        drive.updateHeadingRad();
        botHeading = drive.botHeading;
        if (gamepad1.a) {
            limiter = HIGH_POWER;
        } else if (gamepad1.b) {
            limiter = LOW_POWER;
        }

        if (gamepad1.x) {
            drive.imu.resetYaw();
            fieldCentric = true;
        } else if (gamepad1.y) {
            fieldCentric = false;
        }

         forward = -gamepad1.left_stick_y * limiter;
         strafe = gamepad1.left_stick_x * limiter;
         rotate = gamepad1.right_stick_x * limiter;

        if (Math.abs(gamepad1.left_stick_y) < DEAD_ZONE_P1) {
                forward = 0;

        }

        if (Math.abs(gamepad1.left_stick_x) < DEAD_ZONE_P1) {

                strafe = 0;

        }

        if (Math.abs(gamepad1.right_stick_x) < DEAD_ZONE_P1) {

                rotate = 0;

        }

         fieldForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
         fieldStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);

         if (gamepad1.dpad_up) {
             fieldForward = LOW_POWER;
         } else if (gamepad1.dpad_down) {
             fieldForward = -LOW_POWER;
         }

        if (gamepad1.dpad_right) {
            rotate = LOW_POWER;
        } else if (gamepad1.dpad_left) {
            rotate = -LOW_POWER;
        }

        if (!fieldCentric) {
            drive.driveBot(forward, strafe, rotate);
        } else {
            drive.driveBot(fieldForward, fieldStrafe, rotate);
        }
    }

    public void updateTeleOpTelemetry() {
        telemetry.addData("Limiter: ", limiter);
        telemetry.addData("Heading: ", botHeading);
        telemetry.addData("Field Centric?: ", fieldCentric);
        telemetry.addData("Forward: ", forward);
        telemetry.addData("Strafe: ", strafe);
        telemetry.addData("Rotate:", rotate);
        telemetry.addData("Target slide motor steps:", armMotorSteps);
        telemetry.addData("Actual slide motor steps:", slide.slideMotor.getCurrentPosition());
        telemetry.addData("Slide Power:", slide.slideMotor.getPower());
        telemetry.addData("Target rot motor steps:", rotMotorSteps);
        telemetry.addData("Actual rot motor steps:", slide.rotMotor.getCurrentPosition());
        telemetry.addData("Rot Power:", slide.rotMotor.getPower());
        telemetry.addData("Place:", slide.place);
        telemetry.addData("Turn Rot:", slide.rightRot.getPosition());
        telemetry.addData("Mode:", slide.rotMotor.getMode());



    }
}