package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static java.lang.Math.PI;
import static java.lang.Math.max;

public class MecanumDriveTrain implements DriveTrain {

    public static final int ENCODER_STEPS_PER_REVOLUTION = 1120;
    private final Navigation navigation;
    private final OdometryNavigation oNav;
    private DcMotor leftDriveFront;
    private DcMotor rightDriveFront;
    private DcMotor leftDriveRear;
    private DcMotor rightDriveRear;
    private final HardwareMap hardwareMap;
    private final MecanumParams params;
    public MecanumDriveTrain(HardwareMap hardwareMap, MecanumParams params,
                             Navigation navigation, OdometryNavigation oNav) {
        this.params = params;
        this.hardwareMap = hardwareMap;
        this.navigation = navigation;
        this.oNav = oNav;
    }

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDriveFront = hardwareMap.get(DcMotor.class, Names.LEFT_FRONT);
        rightDriveFront = hardwareMap.get(DcMotor.class, Names.RIGHT_FRONT);
        leftDriveRear = hardwareMap.get(DcMotor.class, Names.LEFT_REAR);
        rightDriveRear = hardwareMap.get(DcMotor.class, Names.RIGHT_REAR);

//        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void driveTo(double x, double y) {
        lookAt(x, y);
        double xBot = navigation.getX();
        double yBot = navigation.getY();

        double distance = Math.sqrt(Math.pow(x - xBot, 2) + Math.pow(y - yBot, 2));
        driveForward(distance);
    }

    @Override
    public void turnAbsolute(double theta) {
        double thetaToTurn = theta - navigation.getTheta();
        turnRelative(thetaToTurn);
    }

    @Override
    public void turnRelative(double thetaToTurn) {
        double startingTheta = navigation.getTheta();
        mecanum(0, 0, thetaToTurn);
        oNav.setTheta(startingTheta + thetaToTurn);
    }

    @Override
    public void lookAt(double x, double y) {
        double theta = calculateDirectionToLook(x, y);
        turnAbsolute(theta);
    }

    double calculateDirectionToLook(double x, double y) {
        double relativeX = x - navigation.getX();
        double relativeY = y - navigation.getY();
        return Math.atan2(relativeY, relativeX);
    }

    @Override
    public void driveForward(double distance) {
        double startingTheta = navigation.getTheta();
        double startingX = navigation.getX();
        double startingY = navigation.getY();
        // set the forward velocity to our speed
        mecanum(0, distance, 0);

        oNav.setX(startingX + distance * Math.cos(startingTheta));
        oNav.setY(startingY + distance * Math.sin(startingTheta));
    }

    @Override
    public void driveLeft(double distance) {
        double startingTheta = navigation.getTheta();
        double startingX = navigation.getX();
        double startingY = navigation.getY();

        mecanum(-distance, 0, 0);

        oNav.setX(startingX + distance * Math.cos(startingTheta + PI/2));
        oNav.setY(startingY + distance * Math.sin(startingTheta + PI/2));
    }

    private void doSleepSeconds(double sleepSeconds) {
        try {
            Thread.sleep(Math.round(sleepSeconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void mecanum(double vtx, double vty, double gamma) {
        // Compute velocities at wheels
        double C = params.getA() + params.getB();
        double vw1 = vty - vtx + gamma * C;
        double vw2 = vty + vtx - gamma * C;
        double vw3 = vty - vtx - gamma * C;
        double vw4 = vty + vtx + gamma * C;

        // from wheel velocities in inches per second to radians per second
        // vwn = r * gamma_n
        double g1 = linearToWheelRotationRadians(vw1);
        double g2 = linearToWheelRotationRadians(vw2);
        double g3 = linearToWheelRotationRadians(vw3);
        double g4 = linearToWheelRotationRadians(vw4);

        // from radians/sec to encoder steps per second.
        double e1 = radiansToRotations(g1);
        double e2 = radiansToRotations(g2);
        double e3 = radiansToRotations(g3);
        double e4 = radiansToRotations(g4);

        double encSteps1 = e1 * ENCODER_STEPS_PER_REVOLUTION;
        double encSteps2 = e2 * ENCODER_STEPS_PER_REVOLUTION;
        double encSteps3 = e3 * ENCODER_STEPS_PER_REVOLUTION;
        double encSteps4 = e4 * ENCODER_STEPS_PER_REVOLUTION;


//        // rotation in encoder steps per second to power.
//        MotorConfigurationType motorType = rightDriveRear.getMotorType();
//        double maxRotationPerSecond = (motorType.getMaxRPM() / 60.0) * motorType.getAchieveableMaxRPMFraction();
//        double p1 = e1 / maxRotationPerSecond;
//        double p2 = e2 / maxRotationPerSecond;
//        double p3 = e3 / maxRotationPerSecond;
//        double p4 = e4 / maxRotationPerSecond;

        int t1 = (int)(rightDriveFront.getCurrentPosition() + encSteps1);
        int t2 = (int)(leftDriveFront.getCurrentPosition() + encSteps2);
        int t3 = (int)(leftDriveRear.getCurrentPosition() + encSteps3);
        int t4 = (int)(rightDriveRear.getCurrentPosition() + encSteps4);

        rightDriveFront.setTargetPosition(t1);
        leftDriveFront.setTargetPosition(t2);
        leftDriveRear.setTargetPosition(t3);
        rightDriveRear.setTargetPosition(t4);

        rightDriveRear.setPower(1);
        rightDriveFront.setPower(1);
        leftDriveRear.setPower(1);
        leftDriveFront.setPower(1);

        while(rightDriveFront.isBusy() ||
                leftDriveFront.isBusy() ||
                leftDriveRear.isBusy() ||
                rightDriveRear.isBusy()) {
            doSleepSeconds(.05);
        }

        rightDriveRear.setPower(0);
        rightDriveFront.setPower(0);
        leftDriveRear.setPower(0);
        leftDriveFront.setPower(0);

//        rightDriveFront.setPower(p1);
//        leftDriveFront.setPower(p2);
//        leftDriveRear.setPower(p3);
//        rightDriveRear.setPower(p4);
    }

    private double linearToWheelRotationRadians(double wheelLinearDistance) {
        return wheelLinearDistance / params.getR();
    }

    private double radiansToRotations(double wheelRotationRadians) {
        return wheelRotationRadians / (2 * PI);
    }
}