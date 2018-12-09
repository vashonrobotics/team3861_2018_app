package org.firstinspires.ftc.teamcode;

public class MecanumParams {
    private final double A;
    private final double B;
    private final double R;

    public double getA() {
        return A;
    }

    public double getB() {
        return B;
    }

    public double getR() {
        return R;
    }

    /**
     * Constructor for the params to a mecanum drive
     * @param A -- Distance between wheels (side to side) divided by 2
     * @param B -- Distance between wheels (front to back) divided by 2
     * @param R -- Wheel radius
     */
    public MecanumParams(double A, double B, double R) {
        this.A = A;

        this.B = B;
        this.R = R;
    }
}