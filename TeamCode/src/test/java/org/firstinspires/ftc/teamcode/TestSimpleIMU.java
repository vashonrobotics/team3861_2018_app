package org.firstinspires.ftc.teamcode;

public class TestSimpleIMU implements SimpleIMU {
    private final double[] imuMeasurements;
    private int currentMeasurement = 0;

    public TestSimpleIMU(double ... imuMeasurements) {
        assert imuMeasurements.length > 0;
        this.imuMeasurements = imuMeasurements;
    }

    @Override
    public void init() {
    }

    @Override
    public double getHeading() {
        double heading =  imuMeasurements[currentMeasurement];
        if(currentMeasurement + 1 < imuMeasurements.length)
            currentMeasurement++;

        return heading;
    }
}
