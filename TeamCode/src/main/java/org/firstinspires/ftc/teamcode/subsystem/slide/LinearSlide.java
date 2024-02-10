package org.firstinspires.ftc.teamcode.subsystem.slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    public static int INCREMENT_STEPS_SLIDE = 40;

    //Rot powers
    public static double MIN_POWER_ROT = 0;
    public static double HOLD_POWER_ROT = 1;
    public static double MAX_POWER_ROT = 1;

    //Rot steps
    public static int MIN_ROT = 0;
    public static int LOW_ROT = 1100;
    public static int MEDIUM_ROT = 1295;
    public static int HIGH_ROT = 930;
    public static int MAX_ROT = 2106;
    public static int INCREMENT_ROT = 10;
    public static int TICKSPERROT = 2520;
    public static double DEGPERTICK = 360.0/TICKSPERROT;
    //Grip pos
    public static double RFLOOR = 0.85;
    public static double LFLOOR = 0.30;
    public static double RPLACE = 0.37;
    public static double LPLACE = 0.78;
    public static double RCLOSE = 0.685;
    public static double ROPEN = 0.5;
    public static double LCLOSE = 0;
    public static double LOPEN = 0.5;
    //motors
    public DcMotorEx slideMotor;
    public DcMotorEx rotMotor;

    //Servos that are a part of the robot
    public Servo leftGripper, rightGripper, leftRot, rightRot, droneServo;


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

        droneServo = hardwareMap.get(Servo.class, "droneLauncher");

        leftRot = hardwareMap.get(Servo.class, "rotL");
        rightRot = hardwareMap.get(Servo.class, "rotR");


    }

    public void setSlide(int steps) {
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(armMotorSteps);
    }

    public void setRot(int steps) {
        rotMotorSteps = Range.clip(steps, MIN_ROT - 200, MAX_ROT + 100);
        rotMotor.setTargetPosition(rotMotorSteps);
    }

    public void setArmPos(int armstep, int rotstep) {
        setRot(rotstep);
        setSlide (armstep);
    }

    public void setAutoPos (int armstep, int rotstep) {
        setArmPos(armMotorSteps,rotstep);
        ElapsedTime safetime = new ElapsedTime();
        safetime.reset();
        while (rotMotor.isBusy() && safetime.time() < 3) {

            if (rotMotorSteps == MIN_ROT && rotMotor.getCurrentPosition() > MIN_ROT){
                if (rotMotor.getCurrentPosition() < 100) {
                    rotMotor.setPower(MAX_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else if (rotMotorSteps == MEDIUM_ROT && rotMotor.getCurrentPosition() < MEDIUM_ROT) {
                if (rotMotor.getCurrentPosition() > 400) {
                    rotMotor.setPower(MAX_POWER_ROT);
                } else {
                    rotMotor.setPower(MAX_POWER_ROT);
                }
            } else if (rotMotorSteps < LOW_ROT && rotMotorSteps > MIN_ROT && rotMotor.getCurrentPosition() < LOW_ROT) {
                if (rotMotor.getCurrentPosition() > 300) {
                    rotMotor.setPower(MAX_POWER_ROT);
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

        if (!place) {
            turnFloorEx();
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
    public void grabR() {
        rightGripper.setPosition(RCLOSE);
    }

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
        if (rotMotor.getCurrentPosition() > HIGH_ROT) {
            turnRot(rightRot, RPLACE + (DEGPERTICK * (1.0/270.0) * (rotMotor.getCurrentPosition() - (1090))));
            turnRot(leftRot, LPLACE - (DEGPERTICK * (1.0/270.0) * (rotMotor.getCurrentPosition() - (1090))));
        } else {
            turnPlace();
        }
    }

    public void turnFloorEx() {
        if (rotMotor.getCurrentPosition() < HIGH_ROT) {
            turnRot(rightRot, RFLOOR + (DEGPERTICK * (1.0/270.0) * (rotMotor.getCurrentPosition())));
            turnRot(leftRot, LFLOOR - (DEGPERTICK * (1.0/270.0) * (rotMotor.getCurrentPosition())));
        } else {
            turnFloor();
        }
    }
    public void turnPlaceAuto() {
        turnRot(rightRot, RPLACE - 0.29);
        turnRot(leftRot, LPLACE + 0.29);
    }




}
