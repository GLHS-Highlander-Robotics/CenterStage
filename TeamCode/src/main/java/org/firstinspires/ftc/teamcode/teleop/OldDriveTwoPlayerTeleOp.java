package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.INCREMENT_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.INCREMENT_STEPS_SLIDE;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.LOW_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.LOW_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MAX_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MEDIUM_ROT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MIN_ROT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    boolean rotated = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry multTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new OldDrive(hardwareMap);
        slide = new LinearSlide(hardwareMap);

        drive.setModes(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        drive.imu.resetYaw();
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
            armMotorSteps = LOW_HEIGHT;
//            slide.turnFloor();
        } else if (gamepad2.b) {
            rotMotorSteps = LOW_ROT;
//            slide.turnPlace();
        } else if (gamepad2.x) {
            rotMotorSteps = MEDIUM_ROT;
            armMotorSteps = MIN_HEIGHT;
        } else if (gamepad2.y) {
            rotMotorSteps = MAX_ROT;
        }

        // Increase arm and rotation steps by increments using p2 sticks
        if (-gamepad2.left_stick_y > DEAD_ZONE_P2) {
            armMotorSteps += INCREMENT_STEPS_SLIDE;
             leftStickPressed = true;
        } else if (-gamepad2.left_stick_y < -DEAD_ZONE_P2) {
            armMotorSteps -= INCREMENT_STEPS_SLIDE;
             leftStickPressed= true;
        } else if (leftStickPressed) {
             leftStickPressed = false;
        }

        if (-gamepad2.right_stick_y > DEAD_ZONE_P2) {
            rotMotorSteps += INCREMENT_ROT;
            rightStickPressed = true;
        } else if (-gamepad2.right_stick_y < -DEAD_ZONE_P2) {
            rotMotorSteps -= INCREMENT_ROT;
            rightStickPressed = true;
        } else if (rightStickPressed) {
            rightStickPressed = false;
        }
        armMotorSteps = Range.clip(armMotorSteps, MIN_HEIGHT, MAX_HEIGHT);
        rotMotorSteps = Range.clip(rotMotorSteps, MIN_ROT, MAX_ROT);
        slide.setArmPos(armMotorSteps,rotMotorSteps);

        //Grab claw with p1 bumpers
//        if (gamepad1.right_trigger > 0.5 && rightGrabbed) {
//            slide.ungrabR();
//        } else if (gamepad1.right_trigger > 0.5 && !rightGrabbed) {
//            slide.grabR();
//        } else if (!rightGrabbed) {
//            rightGrabbed = true;
//        } else {
//            rightGrabbed = false;
//        }
//
//        if (gamepad1.left_trigger > 0.5 && leftGrabbed) {
//            slide.ungrabL();
//        } else if (gamepad1.left_trigger > 0.5 && !leftGrabbed) {
//            slide.grabL();
//        } else if (gamepad1.left_trigger < 0.5 && !leftGrabbed) {
//            leftGrabbed = true;
//        } else if (gamepad1.left_trigger < 0.5 && leftGrabbed) {
//            leftGrabbed = false;
//        }

        //Rotate claw with p2 bumpers
        if (gamepad2.right_trigger > 0.5) {
            slide.place = true;

        } else if (gamepad2.left_trigger > 0.5) {
            slide.place = false;

        }
        //incremental rotation with p2 dpad
//        if (gamepad2.dpad_up) {
//            slide.turnRot(slide.rightRot, slide.rightRot.getPosition() + 0.1);
//            slide.turnRot(slide.leftRot, slide.leftRot.getPosition() - 0.1);
//            rotated = true;
//        } else if (gamepad2.dpad_down) {
//            slide.turnRot(slide.rightRot, slide.rightRot.getPosition() - 0.1);
//            slide.turnRot(slide.leftRot, slide.leftRot.getPosition() + 0.1);
//            rotated = true;
//        } else if (rotated) {
//            rotated = false;
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



    }
}