package org.firstinspires.ftc.teamcode;

public abstract class AbstractDifferentialDriveTrain implements DriveTrain {
    protected final SimpleOutput output;
    protected final OdometryNavigation oNav;
    protected final Navigation navigation;

    public AbstractDifferentialDriveTrain(SimpleOutput output, OdometryNavigation oNav, Navigation navigation) {
        this.output = output;
        this.oNav = oNav;
        this.navigation = navigation;
    }

    @Override
    public void init() {}

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
        double wheelRadians = thetaToTurn * getDriveLeverArmLength() / getWheelRadius();
        int stepsToTurn = getStepsToTurn(wheelRadians);

        turnWheels(stepsToTurn, -stepsToTurn);
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

    private int getStepsToTurn(double wheelRadians) {
        double wheelTurns = wheelRadians / (2 * Math.PI);

        int steps =  (int)Math.round(wheelTurns * getStepsPerWheelRotation());
        String message = String.format("Radians: %f, Steps: %d", wheelRadians, steps);
        output.write("Turning wheels", message);

        return steps;
    }

    abstract void turnWheels(int rightSteps, int leftSteps);
    abstract protected double getStepsPerWheelRotation();
    abstract protected double getWheelRadius();
    abstract protected double getDriveLeverArmLength();
}