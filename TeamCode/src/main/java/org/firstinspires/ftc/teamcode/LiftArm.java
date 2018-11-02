package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LiftArm implements hardwareSubsystem {
    private DcMotor top;
    private DcMotor bottom;
    public Servo camServo;
    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION = 1120;
    private static final double WHEEL_RADIUS_INCHES = 0.955;
    private HardwareMap hardwareMap;
    private DigitalChannel topLimit;
    private DigitalChannel bottomLimit;

    public LiftArm(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
    }
    public void init() {
        top=hardwareMap.get(DcMotor.class, Names.LIFT_TOP);
        bottom=hardwareMap.get(DcMotor.class, Names.LIFT_BOTTOM);
        camServo=hardwareMap.get(Servo.class, Names.CAM_SERVO);

        top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topLimit = hardwareMap.get(DigitalChannel.class, Names.TOP_LIMIT);
        topLimit.setMode(DigitalChannel.Mode.INPUT);

        bottomLimit = hardwareMap.get(DigitalChannel.class, Names.BOTTOM_LIMIT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);

        runToBottomLimit();
    }

    public void raise() {
        runToTopLimit();
        turnWheels(getStepsToTurn(20),top);
        camServo.setPosition(0);
    }

    public void runToBottomLimit() {
        runToSwitch(this.bottomLimit, 0.5);
    }

    public void runToTopLimit() {
        runToSwitch(this.topLimit, -0.5);
    }

    private void runToSwitch(DigitalChannel limitSwitch, double power) {
        bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottom.setPower(power);

        while(!limitSwitch.getState()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            }
        }

        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private int getStepsToTurn(double distance) {
        double wheelRadians = distance / WHEEL_RADIUS_INCHES;
        double wheelTurns = wheelRadians / (2 * Math.PI);

        int steps =  (int)Math.round(wheelTurns * ENCODER_STEPS_PER_WHEEL_ROTATION);

        return steps;
    }

    private void turnWheels( int steps,DcMotor selected) {

        selected.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        selected.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        selected.setTargetPosition(steps);
        selected.setPower(0.05);

        while(selected.isBusy()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                break;
            }
        }
    }

    public void lower() {
        runToTopLimit();
        turnWheels(getStepsToTurn(-20),top);
        camServo.setPosition(1);
    }
}
