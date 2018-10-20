package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FourWheelDifferentialDriveTrain extends AbstractDifferentialDriveTrain {
    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION = 1120;

    // L is the distance between the center of rotation on the bot and the wheel
    private static final double L_INCHES = 15.5 / 2;
    private static final double WHEEL_RADIUS_INCHES = 7.10 / 2;

    private DcMotor leftDriveRear = null;
    private DcMotor leftDriveFront = null;

    private DcMotor rightDriveRear = null;
    private DcMotor rightDriveFront = null;

    private HardwareMap hardwareMap;

    public FourWheelDifferentialDriveTrain(HardwareMap hardwareMap, Navigation navigation,
                                           OdometryNavigation oNav,
                                           SimpleOutput output) {
        super(output, oNav, navigation);
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        super.init();

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

    protected double getWheelRadius() { return WHEEL_RADIUS_INCHES; }
    protected double getDriveLeverArmLength () { return L_INCHES; }
    protected double getStepsPerWheelRotation() { return ENCODER_STEPS_PER_WHEEL_ROTATION; }

    protected void turnWheels(int rightSteps, int leftSteps) {
        String message = String.format("L %d, R %s", rightSteps, leftSteps);
        output.write("Turning wheels", message);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int leftFrontSteps = leftDriveFront.getCurrentPosition() + leftSteps;
        int leftRearSteps = leftDriveRear.getCurrentPosition() + leftSteps;
        int rightFrontSteps = rightDriveFront.getCurrentPosition() + rightSteps;
        int rightRearSteps = rightDriveRear.getCurrentPosition() + rightSteps;

        leftDriveFront.setTargetPosition(leftFrontSteps);
        leftDriveRear.setTargetPosition(leftRearSteps);
        rightDriveFront.setTargetPosition(rightFrontSteps);
        rightDriveRear.setTargetPosition(rightRearSteps);
        leftDriveFront.setPower(0.05);
        leftDriveRear.setPower(0.05);
        rightDriveFront.setPower(0.05);
        rightDriveRear.setPower(0.05);

        while(rightDriveFront.isBusy() || rightDriveRear.isBusy() || leftDriveFront.isBusy() || leftDriveRear.isBusy()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                break;
            }
        }
        output.write("Turning wheels", "Finished");
    }
}
