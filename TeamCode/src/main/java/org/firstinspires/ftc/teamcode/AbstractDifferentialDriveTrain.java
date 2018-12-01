package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.OrientationUtils;

public abstract class AbstractDifferentialDriveTrain implements DriveTrain {
    protected final SimpleOutput output;
    protected final OdometryNavigation oNav;
    protected final Navigation navigation;
    private final SimpleIMU simpleIMU;
    protected HardwareMap hardwareMap;
    protected double totalErrorProportion = 1;
    protected int errorCount = 1;

    public AbstractDifferentialDriveTrain(SimpleOutput output,
                                          OdometryNavigation oNav,
                                          Navigation navigation,
                                          SimpleIMU simpleIMU,
                                          HardwareMap hardwareMap) {
        this.output = output;
        this.oNav = oNav;
        this.navigation = navigation;
        this.hardwareMap = hardwareMap;
        this.simpleIMU = simpleIMU;
    }

    @Override
    public void init() {
        simpleIMU.init();
    }

    @Override
    public void driveTo(double x, double y) {
        lookAt(x, y);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        double xBot = navigation.getX();
        double yBot = navigation.getY();

        double distance = Math.sqrt(Math.pow(x - xBot, 2) + Math.pow(y - yBot, 2));
        driveForward(distance);
    }

    public void driveForward(double distance) {
        double startingTheta = navigation.getTheta();
        double startingX = navigation.getX();
        double startingY = navigation.getY();
        double wheelRadians = distance / getWheelRadius();
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

    @Override
    public void turnRelative(double thetaToTurn) {
        double startingTheta = navigation.getTheta();
        double imuCurrent = simpleIMU.getHeading();
        double imuTarget = OrientationUtils.plus(imuCurrent, thetaToTurn);
        double imuTurnAmount = OrientationUtils.minus(imuTarget, imuCurrent);
        boolean first = true;

        do {
            // double errorCorrectionFactor = errorCount / totalErrorProportion;
            doTurn(imuTurnAmount);

            // wait for the IMU to settle
            doWait(250);

            // update the error
            imuCurrent = simpleIMU.getHeading();
            double newImuTurnAmount = OrientationUtils.minus(imuTarget, imuCurrent);

            double actuallyTurned = OrientationUtils.minus(imuTurnAmount, newImuTurnAmount);
            double errorProportion = Math.abs(actuallyTurned / imuTurnAmount);

            String msg = String.format("%b, %f, %f, %f, %f",
                    first,
                    thetaToTurn,
                    imuTurnAmount,
                    actuallyTurned,
                    errorProportion * 100);
            output.write("Turn error", msg);
            RobotLog.i(msg);
            totalErrorProportion += errorProportion;
            errorCount++;
            first = false;
            imuTurnAmount = newImuTurnAmount;

        } while (Math.abs(imuTurnAmount) > 0.05);

        oNav.setTheta(startingTheta + thetaToTurn);
    }

    private void doWait(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void doTurn(double thetaToTurn) {
        double wheelRadians = thetaToTurn * getDriveLeverArmLength() / getWheelRadius();
        int stepsToTurn = getStepsToTurn(wheelRadians);

        turnWheels(stepsToTurn, -stepsToTurn);
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

    private int getStepsToTurn(double wheelRadians) {
        double wheelTurns = wheelRadians / (2 * Math.PI);

        int steps =  (int)Math.round(wheelTurns * getStepsPerWheelRotation());
//        String message = String.format("Radians: %f, Steps: %d", wheelRadians, steps);
//        output.write("Turning wheels", message);

        return steps;
    }

    abstract void turnWheels(int rightSteps, int leftSteps);
    abstract protected double getStepsPerWheelRotation();
    abstract protected double getWheelRadius();
    abstract protected double getDriveLeverArmLength();
}