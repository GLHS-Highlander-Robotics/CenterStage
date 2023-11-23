package org.firstinspires.ftc.teamcode.subsystem.slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static int LOW_HEIGHT = 515;
    public static int MEDIUM_HEIGHT = 1600;
    public static int MAX_HEIGHT = 2760;
    public static int INCREMENT_STEPS_SLIDE = 20;

    //Rot powers
    public static double MIN_POWER_ROT = 0;
    public static double HOLD_POWER_ROT = 1;
    public static double MAX_POWER_ROT = 0.50;

    //Rot steps
    public static int MIN_ROT = 0;
    public static int LOW_ROT = 350;
    public static int MEDIUM_ROT = 440;
    public static int MAX_ROT = 540;
    public static int INCREMENT_ROT = 3;
    //Grip pos
    public static double RFLOOR = 0.9;
    public static double LFLOOR = 0.29;
    public static double RPLACE = 0.31;
    public static double LPLACE = 0.85;
    public static double RCLOSE = 0.685;
    public static double ROPEN = 0.5;
    public static double LCLOSE = 0;
    public static double LOPEN = 0.5;
    //motors
    public DcMotorEx slideMotor;
    public DcMotorEx rotMotor;

    //Servos that are a part of the robot
    public Servo leftGripper, rightGripper, leftRot, rightRot;


    private int armMotorSteps = 0;
    private int rotMotorSteps = 0;

    public boolean place = false;

    public LinearSlide(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slidemotor");

        slideMotor.setDirection(DcMotorEx.Direction.FORWARD);
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


        leftGripper = hardwareMap.get(Servo.class, "gripL");
        rightGripper = hardwareMap.get(Servo.class, "gripR");

        leftRot = hardwareMap.get(Servo.class, "rotL");
        rightRot = hardwareMap.get(Servo.class, "rotR");


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

    public void setAutoPos (int armstep, int rotstep) {
        setArmPos(armMotorSteps,rotstep);
        while (rotMotor.isBusy()) {
            if (rotMotorSteps == MIN_ROT && rotMotor.getCurrentPosition() > MIN_ROT){
                if (rotMotor.getCurrentPosition() < 100) {
                    rotMotor.setPower(MIN_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else if (rotMotorSteps == MEDIUM_ROT && rotMotor.getCurrentPosition() < MEDIUM_ROT) {
                if (rotMotor.getCurrentPosition() > 400) {
                    rotMotor.setPower(MIN_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else if (rotMotorSteps < LOW_ROT && rotMotorSteps > MIN_ROT && rotMotor.getCurrentPosition() < LOW_ROT) {
                if (rotMotor.getCurrentPosition() > 300) {
                    rotMotor.setPower(MIN_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else {
                rotMotor.setPower(MAX_POWER_ROT);
            }
        }
        rotMotor.setPower(HOLD_POWER_ROT);
        if (rotMotor.getCurrentPosition() == MIN_ROT) {
            rotMotor.setPower(MIN_POWER_ROT);
        }
        setArmPos(armstep,rotstep);
        while (slideMotor.isBusy()) {
            slideMotor.setPower(MAX_POWER_SLIDE);
        }
        slideMotor.setPower(HOLD_POWER_SLIDE);
        if (slideMotor.getCurrentPosition() == MIN_HEIGHT) {
            slideMotor.setPower(MIN_POWER_SLIDE);
        }
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
            if (rotMotorSteps == MIN_ROT && rotMotor.getCurrentPosition() > MIN_ROT){
                if (rotMotor.getCurrentPosition() < 100) {
                    rotMotor.setPower(MIN_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else if (rotMotorSteps == MEDIUM_ROT && rotMotor.getCurrentPosition() < MEDIUM_ROT) {
                if (rotMotor.getCurrentPosition() > 400) {
                    rotMotor.setPower(MIN_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else if (rotMotorSteps == LOW_ROT && rotMotor.getCurrentPosition() < LOW_ROT) {
                if (rotMotor.getCurrentPosition() > 300) {
                    rotMotor.setPower(MIN_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else {
                rotMotor.setPower(MAX_POWER_ROT);
            }
        } else {
            rotMotor.setPower(HOLD_POWER_ROT);
            if (rotMotor.getCurrentPosition() == MIN_ROT) {
                rotMotor.setPower(MIN_POWER_ROT);
            }
        }

        if (place == false) {
            turnFloor();
        } else {
            turnPlaceEx();

        }
    }
//Grab functions
    public void ungrabL() {
        leftGripper.setPosition(LOPEN);
    }
    public void grabL() {
        leftGripper.setPosition(LCLOSE);
    }
    public void ungrabR() {
        rightGripper.setPosition(ROPEN);
    }
    public void grabR() { rightGripper.setPosition(RCLOSE); }
    public void ungrabAll() {
        ungrabL();
        ungrabR();
    }
    public void grabAll() {
        grabL();
        grabR();
    }
    public void turnRot(Servo rotX, double ticks) {
        rotX.setPosition(Range.clip(ticks, 0, 1));

    }

    public void turnFloor() {
        turnRot(rightRot, RFLOOR);
        turnRot(leftRot, LFLOOR);
    }

    public void turnPlace() {
        turnRot(rightRot, RPLACE);
        turnRot(leftRot, LPLACE);
    }

    public void turnPlaceEx() {
        if (rotMotor.getCurrentPosition() > LOW_ROT - 10) {
            turnRot(rightRot, RPLACE + (0.5/270.0) * (rotMotor.getCurrentPosition() - LOW_ROT));
            turnRot(leftRot, LPLACE - (0.5/270.0) * (rotMotor.getCurrentPosition() - LOW_ROT));
        } else {
            turnPlace();
        }
    }
    public void turnPlaceAuto() {
        turnRot(rightRot, RPLACE - 0.29);
        turnRot(leftRot, LPLACE + 0.29);
    }




}
