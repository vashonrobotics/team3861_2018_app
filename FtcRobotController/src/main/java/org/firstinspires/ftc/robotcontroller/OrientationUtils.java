package org.firstinspires.ftc.robotcontroller;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OrientationUtils {
    public static double minus(double left, double right) {
        double result = left - right;
        return AngleUnit.normalizeRadians(result);
    }

    public static double plus(double left, double right) {
        double result = left + right;
        return AngleUnit.normalizeRadians(result);
    }
}
