package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DifferentialDriveTrain extends AbstractDifferentialDriveTrain {

    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION = 1120;

    // L is the distance between the center of rotation on the bot and the wheel
    private static final double L_INCHES = 15.0 / 2;
    private static final double WHEEL_RADIUS_INCHES = 7.25 / 2;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public DifferentialDriveTrain(HardwareMap hardwareMap,
                                  Navigation navigation,
                                  OdometryNavigation oNav,
                                  SimpleIMU simpleIMU,
                                  SimpleOutput output) {
        super(output, oNav, navigation, simpleIMU, hardwareMap);
    }

    public void init() {
        super.init();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, Names.LEFT_REAR);
        rightDrive = hardwareMap.get(DcMotor.class, Names.RIGHT_REAR);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected double getWheelRadius() { return WHEEL_RADIUS_INCHES; }
    protected double getDriveLeverArmLength () { return L_INCHES; }
    protected double getStepsPerWheelRotation() { return ENCODER_STEPS_PER_WHEEL_ROTATION; }

    protected void turnWheels(int rightSteps, int leftSteps) {
        String message = String.format("L %d, R %s", rightSteps, leftSteps);
        output.write("Turning wheels", message);

        int currentLeftPosition = leftDrive.getCurrentPosition();
        int absoluteLeft = currentLeftPosition + leftSteps;
        int currentRightPosition = rightDrive.getCurrentPosition();
        int absoluteRight = currentRightPosition + rightSteps;

        leftDrive.setTargetPosition(absoluteLeft);
        rightDrive.setTargetPosition(absoluteRight);
        leftDrive.setPower(0.25);
        rightDrive.setPower(0.25);

        while(rightDrive.isBusy() || leftDrive.isBusy()) {
            try {
                currentLeftPosition = leftDrive.getCurrentPosition();
                currentRightPosition = rightDrive.getCurrentPosition();
                RobotLog.i("Left %d, right %d", currentLeftPosition, currentRightPosition);
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                break;
            }
        }
        output.write("Turning wheels", "Finished");
    }
}
