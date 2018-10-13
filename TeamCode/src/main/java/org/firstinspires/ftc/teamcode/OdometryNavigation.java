package org.firstinspires.ftc.teamcode;

public class OdometryNavigation implements Navigation {
    private double x;
    private double y;
    private double theta;

    public OdometryNavigation(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    @Override
    public double getTheta() {
        return theta;
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }


}
