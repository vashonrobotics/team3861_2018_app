package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestDifferentialDriveTrain extends DifferentialDriveTrain {
    static OdometryNavigation nav = new OdometryNavigation(16, 16, Math.PI/4);

    public int getLastRightWheelSteps() {
        return lastRightWheelSteps;
    }

    public static OdometryNavigation getNav() {
        return nav;
    }

    public int getLastLeftWheelSteps() {
        return lastLeftWheelSteps;
    }

    private int lastRightWheelSteps = 0;
    private int lastLeftWheelSteps = 0;

    static OdometryNavigation resetNav(double x, double y, double theta) {
        nav.setX(x);
        nav.setY(y);
        nav.setTheta(theta);

        return nav;
    }

    public TestDifferentialDriveTrain(double x, double y, double theta,
                                      double ... imuMeasurements) {
        super(null, resetNav(x, y, theta), nav,
                new TestSimpleIMU(imuMeasurements),
                new PrintingSimleOutput());
    }

    @Override
    public void init() {}

    @Override
    protected void turnWheels(int rightSteps, int leftSteps) {
        lastRightWheelSteps = rightSteps;
        lastLeftWheelSteps = leftSteps;
    }
}
