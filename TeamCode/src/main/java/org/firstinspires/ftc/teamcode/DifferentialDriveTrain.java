package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DifferentialDriveTrain implements DriveTrain {
    private static final double COUNTS_PER_MOTOR_REV = 4 ;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 72 ;     // This is < 1.0 if geared UP

    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION =
            COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;

    // L is the distance between the center of rotation on the bot and the wheel
    private static final double L_INCHES = 16.0;
    private static final double WHEEL_RADIUS_INCHES = 3.5;
    private final Telemetry telemetry;
    private final OdometryNavigation oNav;
    private Navigation navigation;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private HardwareMap hardwareMap;

    public DifferentialDriveTrain(HardwareMap hardwareMap, Navigation navigation,
                                  OdometryNavigation oNav,
                                  Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.navigation = navigation;
        this.telemetry = telemetry;
        this.oNav = oNav;
    }

    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        */

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void driveTo(double x, double y) {
        lookAt(x, y);
        double xBot = navigation.getX();
        double yBot = navigation.getY();

        double distance = Math.sqrt(Math.pow(x - xBot, 2) + Math.pow(y - yBot, 2));
        driveForward(distance);

    }

    public void driveForward(double distance) {
        double startingTheta = navigation.getTheta();
        double startingX = navigation.getX();
        double startingY = navigation.getY();
        double wheelRadians = distance / WHEEL_RADIUS_INCHES;
        int stepsToTurn = getStepsToTurn(wheelRadians);
        turnWheels(stepsToTurn, stepsToTurn);
        /*
        Set up for updating new coordinates for where the location of the robot
        is for navigating the game field.
        */
        oNav.setX(startingX + distance * Math.cos(startingTheta));
        oNav.setY(startingY + distance * Math.sin(startingTheta));
    }

    @Override
    public void turnAbsolute(double theta) {
        double thetaToTurn = theta - navigation.getTheta();
        turnRelative(thetaToTurn);
    }

    private void turnRelative(double thetaToTurn) {
        double startingTheta = navigation.getTheta();
        double wheelRadians = thetaToTurn * L_INCHES / WHEEL_RADIUS_INCHES;
        int stepsToTurn = getStepsToTurn(wheelRadians);

        turnWheels(-stepsToTurn, stepsToTurn);
        oNav.setTheta(startingTheta + thetaToTurn);
    }

    private void turnWheels(int rightSteps, int leftSteps) {
        String message = String.format("L %d, R %s", rightSteps, leftSteps);
        telemetry.addData("Turning wheels", message);
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition(leftSteps);
        rightDrive.setTargetPosition(rightSteps);
        leftDrive.setPower(0.05);
        rightDrive.setPower(0.05);

        while(rightDrive.isBusy() || leftDrive.isBusy()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                break;
            }
        }
        telemetry.addData("Turning wheels", "Finished");
    }

    private int getStepsToTurn(double wheelRadians) {
        double wheelTurns = wheelRadians / (2 * Math.PI);

        int steps =  (int)Math.round(wheelTurns * ENCODER_STEPS_PER_WHEEL_ROTATION);
        String message = String.format("Radians: %f, Steps: %d", wheelRadians, steps);
        telemetry.addData("Turning wheels", message);
        telemetry.update();

        return steps;
    }

    @Override
    public void lookAt(double x, double y) {
        double relativeX = x - navigation.getX();
        double relativeY = y - navigation.getY();
        double theta = Math.atan2(relativeY, relativeX);
        turnAbsolute(theta);
    }
}