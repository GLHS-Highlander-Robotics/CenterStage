package org.firstinspires.ftc.teamcode.subsystem.slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class LinearSlide {

    //Slide powers
    public static double MIN_POWER_SLIDE = 0;
    public static double HOLD_POWER_SLIDE = 0.1;
    public static double MAX_POWER_SLIDE = 1;

    //Slide heights
    public static int MIN_HEIGHT = 0;
    public static int LOW_HEIGHT = 800;
    public static int MEDIUM_HEIGHT = 1600;
    public static int MAX_HEIGHT = 2760;
    public static int INCREMENT_STEPS_SLIDE = 20;

    //Rot powers
    public static double MIN_POWER_ROT = 0;
    public static double HOLD_POWER_ROT = 0.3;
    public static double MAX_POWER_ROT = 0.55;

    //Rot steps
    public static int MIN_ROT = 0;
    public static int LOW_ROT = 440;
    public static int MEDIUM_ROT = 300;
    public static int MAX_ROT = 540;
    public static int INCREMENT_ROT = 5;
    //Grip pos
    public static double GRIP_MIN = 0.0;
    public static double GRIP_MAX = 1.0;
    public static double TURN_MAX = 1.0;
    public static double TURN_MIN = 0.0;
    //motors
    public DcMotorEx slideMotor;
    public DcMotorEx rotMotor;

    //Servos that are a part of the robot
    public Servo leftGripper, rightGripper, leftRot, rightRot;


    private int armMotorSteps = 0;
    private int rotMotorSteps = 0;

    public LinearSlide(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "motor_slide");

        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        slideMotor.setTargetPosition(MIN_HEIGHT);
        slideMotor.setPower(MAX_POWER_SLIDE);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rotMotor = hardwareMap.get(DcMotorEx.class, "rotmotor");
        rotMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rotMotor.setTargetPosition(MIN_ROT);
        rotMotor.setPower(MAX_POWER_ROT);
        rotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


//        leftGripper = hardwareMap.get(Servo.class, "gripL");
//        rightGripper = hardwareMap.get(Servo.class, "gripR");
//
//        leftRot = hardwareMap.get(Servo.class, "rotL");
//        rightRot = hardwareMap.get(Servo.class, "rotR");


    }

    public void setSlide(int steps) {
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(armMotorSteps);
    }

    public void setRot(int steps) {
        rotMotorSteps = Range.clip(steps, MIN_ROT, MAX_ROT);
        rotMotor.setTargetPosition(rotMotorSteps);
    }

    public void setArmPos(int armstep, int rotstep) {
        setRot(rotstep);
        setSlide (armstep);
    }

    public void update() {
        // If motor is busy, run motor at max power.
        // If motor is not busy, hold at current position or stop at lowest height
        if (slideMotor.isBusy()) {
            slideMotor.setPower(MAX_POWER_SLIDE);
        } else {
            slideMotor.setPower(HOLD_POWER_SLIDE);
            if (slideMotor.getCurrentPosition() == MIN_HEIGHT) {
                slideMotor.setPower(MIN_POWER_SLIDE);
            }
        }
        if (rotMotor.isBusy()) {
            rotMotor.setPower(MAX_POWER_ROT);
        } else {
            rotMotor.setPower(HOLD_POWER_ROT);
            if (rotMotor.getCurrentPosition() == MIN_HEIGHT) {
                rotMotor.setPower(MIN_POWER_ROT);
            }
        }
    }
//Grab functions
    public void ungrabL() {
        leftGripper.setPosition(GRIP_MIN);
    }
    public void grabL() {
        leftGripper.setPosition(GRIP_MAX + 0.1);
    }
    public void ungrabR() {
        rightGripper.setPosition(GRIP_MAX);
    }
    public void grabR() {
        rightGripper.setPosition(GRIP_MIN - 0.1);
    }
    public void ungrabAll() {
        leftGripper.setPosition(GRIP_MIN);
        rightGripper.setPosition(GRIP_MAX);
    }
    public void grabAll() {
        leftGripper.setPosition(GRIP_MAX + 0.1);
        rightGripper.setPosition(GRIP_MIN - 0.1);
    }
    public void turnRot(Servo rotX, double ticks){
        rotX.setPosition(Range.clip(ticks, TURN_MIN, TURN_MAX));

    }

    public void turnFloor() {
        turnRot(rightRot, TURN_MIN);
        turnRot(leftRot, TURN_MAX);
    }

    public void turnPlace() {
        turnRot(rightRot, TURN_MAX - 0.4);
        turnRot(leftRot, TURN_MIN + 0.4);
    }




}
