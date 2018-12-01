package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveTrain implements DriveTrain {

    private final Navigation navigation;
    private DcMotor leftDriveFront;
    private DcMotor rightDriveFront;
    private DcMotor leftDriveRear;
    private DcMotor rightDriveRear;
    private final HardwareMap hardwareMap;
// for servos, we'll have "latchServo." other motors: "intakeMotor,", "armMotor," "liftMotor," and "extendMotor."
    public MecanumDriveTrain(HardwareMap hardwareMap, Navigation navigation) {
        this.hardwareMap = hardwareMap;
        this.navigation = navigation;
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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    }
}



//package org.firstinspires.ftc.teamcode;
//
//        import com.qualcomm.robotcore.hardware.DcMotor;
//
//        import java.util.ArrayList;
//        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//
///**
// * Created by Caz on 9/30/2017.
// */
//
//public class DriveTrain {
//
//    //    baseMotorArray goes in order: frontLeft, frontRight, backLeft, backRight
//    public static void nonMecanum(ArrayList baseMotorArray, double power[]) {
//    }
//
//    public static void mecanum(ArrayList baseMotorArray, double x, double y, double turn, boolean frontIsInTheDirectionOfTheWheels){
//        double power = maxUnit(Math.sqrt((x * x) + (y * y)));
////        double radianAngle = 0;
//        double radianAngle = Math.atan2(y, x) - Math.PI * 1/4;
////
//        if (Math.abs(power) + Math.abs(turn) > 1)
//        {
//            power = power /(Math.abs(power) + Math.abs(turn));
//            turn = Math.signum(turn) * (1 - Math.abs(power));
//        }
//
//        double motorPower[] = {
//                (Math.cos(radianAngle) * power) + turn, // frontLeft
//                (Math.sin(radianAngle) * power) - turn*(frontIsInTheDirectionOfTheWheels ? 1:-1), // frontRight
//                (Math.sin(radianAngle) * power) + turn*(frontIsInTheDirectionOfTheWheels ? 1:-1), // backLeft
//                (Math.cos(radianAngle) * power) - turn  // backRight
//        };
//
//        for (int i = 0; i < baseMotorArray.size(); i++) {
//            ((DcMotor) baseMotorArray.get(i)).setPower(maxUnit(motorPower[i]));
//        }
//    }
//
//    private static double maxUnit(final double input) {
//        return input > 1 ? 1 : input < -1 ? -1 : input;
//    }
//    public static void turn(ArrayList baseMotorArray, double angle, double wheelWidthBetweenWheels, double wheelHeightBetweenWheels){
//        // angle is in degrees
//        // wheel distances are in mm
//        // WARNING: leaves motors in run to position mode and doesn't wait for them to run to the position
//        double distanceToTravel = 2*Math.PI*Math.sqrt(Math.pow(wheelHeightBetweenWheels/2,2)+Math.pow(wheelWidthBetweenWheels/2,2))*angle/360;
//        final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
//        final double     DRIVE_GEAR_REDUCTION = 0.5 ;     // This is < 1.0 if geared UP
//        final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
//        final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                (WHEEL_DIAMETER_MM * 3.1415);
//        for (int i = 0; i < baseMotorArray.size(); i++) {
//            DcMotor motor = ((DcMotor) baseMotorArray.get(i));
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            int sideMultiplier = i % 2 == 0 ? 1 : -1;
//            motor.setPower(0.4*sideMultiplier*Math.signum(angle));
//        }
//        while (Math.abs((int) (distanceToTravel * COUNTS_PER_MM)) > Math.abs(((DcMotor) baseMotorArray.get(0)).getCurrentPosition())) {
//        }
//        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
//    }
//}