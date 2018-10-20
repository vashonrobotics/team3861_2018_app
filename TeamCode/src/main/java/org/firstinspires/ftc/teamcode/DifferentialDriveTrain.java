package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DifferentialDriveTrain extends AbstractDifferentialDriveTrain {
    private static final double COUNTS_PER_MOTOR_REV = 4 ;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 72 ;     // This is < 1.0 if geared UP

    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION =
            COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;

    // L is the distance between the center of rotation on the bot and the wheel
    private static final double L_INCHES = 16.5 / 2;
    private static final double WHEEL_RADIUS_INCHES = 7.25 / 2;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private HardwareMap hardwareMap;

    public DifferentialDriveTrain(HardwareMap hardwareMap, Navigation navigation,
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
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        */

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected double getWheelRadius() { return WHEEL_RADIUS_INCHES; }
    protected double getDriveLeverArmLength () { return L_INCHES; }
    protected double getStepsPerWheelRotation() { return ENCODER_STEPS_PER_WHEEL_ROTATION; }

    protected void turnWheels(int rightSteps, int leftSteps) {
        String message = String.format("L %d, R %s", rightSteps, leftSteps);
        output.write("Turning wheels", message);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition(leftSteps);
        rightDrive.setTargetPosition(rightSteps);
        leftDrive.setPower(0.05);
        rightDrive.setPower(0.05);

        while(rightDrive.isBusy() || leftDrive.isBusy()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                break;
            }
        }
        output.write("Turning wheels", "Finished");
    }
}
