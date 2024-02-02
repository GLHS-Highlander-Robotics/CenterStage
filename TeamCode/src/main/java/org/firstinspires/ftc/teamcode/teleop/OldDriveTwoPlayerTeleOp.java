package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.HIGH_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.INCREMENT_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.INCREMENT_STEPS_SLIDE;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.LOW_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.LOW_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MEDIUM_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MEDIUM_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MIN_ROT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystem.drive.OldDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;

@Config
@TeleOp (name = "OGTwoPlayerTeleop")
public class OldDriveTwoPlayerTeleOp extends LinearOpMode {
    public static double HIGH_POWER = 1.0;
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

    boolean leftGrabbed = false;
    boolean rightGrabbed = false;
    boolean detectedR = false;
    boolean detectedL = false;
    boolean detectedRot = false;
    boolean detectedRotTrig = false;
    boolean detectedExtendo = false;
    boolean extendoOn = false;
    boolean rotTrigged = false;

    @Override
    public void runOpMode() throws InterruptedException {

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
            slide.turnFloorEx();
            slide.place = false;
            extendoOn = false;
        } else if (gamepad2.b) {
            rotMotorSteps = LOW_ROT;
            armMotorSteps = MIN_HEIGHT;
            slide.turnPlaceEx();
            slide.place=true;
            extendoOn = false;
        } else if (gamepad2.x) {
            rotMotorSteps = MEDIUM_ROT+30;
            armMotorSteps = MIN_HEIGHT;
            slide.turnPlaceEx();
            slide.place=true;
            extendoOn = false;
        } else if (gamepad2.y) {
            rotMotorSteps = HIGH_ROT;
            armMotorSteps = MIN_HEIGHT;
            extendoOn = false;
        }else
        if (gamepad2.dpad_left){
            rotMotorSteps = 500;
            armMotorSteps=MIN_HEIGHT;
            extendoOn = false;
        }
        // Increase arm and rotation steps by increments using p2 sticks
        if (-gamepad2.left_stick_y > DEAD_ZONE_P2) {
            if (!extendoOn) {
                armMotorSteps += INCREMENT_STEPS_SLIDE;
            } else {armMotorSteps += (INCREMENT_STEPS_SLIDE * 2);}
            leftStickPressed = true;
        } else if (-gamepad2.left_stick_y < -DEAD_ZONE_P2) {
            if (!extendoOn) {
                armMotorSteps -= INCREMENT_STEPS_SLIDE;
            } else {armMotorSteps -= (INCREMENT_STEPS_SLIDE * 2);}
            leftStickPressed = true;
        } else if (leftStickPressed) {
            leftStickPressed = false;
        }


        if (!detectedExtendo) {
            if (gamepad2.dpad_right && !extendoOn) {
                if (slide.slideMotor.getCurrentPosition() < 500) {
                    extendoOn = true;
                    detectedExtendo = true;
                }
            } else if (gamepad2.dpad_right && extendoOn) {
                extendoOn = false;
                detectedExtendo = true;
            }
        } else {
            if (!gamepad2.dpad_right) {
                detectedExtendo = false;
            }
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

        if (extendoOn) {
            rotMotorSteps = Range.clip((int)((( Math.toDegrees(Math.acos(9/(9 + 4 + (4 * Range.clip(armMotorSteps - 520, 0, 1000000000)/340.0)))))-50)/(LinearSlide.DEGPERTICK) ), 0, 500);
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
            slide.turnRot(slide.droneServo, 0.60);
        }

    }






    public void updateDriveByGamepad() {
        drive.updateHeadingRad();
        botHeading = drive.botHeading;


        if (gamepad1.x) {
            drive.imu.resetYaw();
            fieldCentric = true;
        } else if (gamepad1.y) {
            fieldCentric = false;
        }

         forward = -gamepad1.left_stick_y * limiter;
         strafe = gamepad1.left_stick_x * limiter;
         rotate = gamepad1.right_stick_x * limiter;



        if (Math.abs(gamepad1.left_stick_y) < DEAD_ZONE_P1) { forward = 0; }

        if (Math.abs(gamepad1.left_stick_x) < DEAD_ZONE_P1) { strafe = 0; }

        if (Math.abs(gamepad1.right_stick_x) < DEAD_ZONE_P1) { rotate = 0; }

        if (gamepad1.a) {
            forward = LOW_POWER;
        } else if (gamepad1.b) {
            forward = -LOW_POWER;
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
        telemetry.addData("Extendo:", extendoOn);



    }
}