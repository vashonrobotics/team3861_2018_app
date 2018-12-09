package org.firstinspires.ftc.teamcode;

public interface DriveTrain {
    void init();
    void driveTo(double x,double y);
    void turnAbsolute(double theta);

    void turnRelative(double thetaToTurn);

    void lookAt(double x, double y);

    void driveForward(double distance);
    void driveLeft(double distance);
}