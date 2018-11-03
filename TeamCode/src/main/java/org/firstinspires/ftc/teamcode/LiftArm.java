package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;


public class LiftArm implements hardwareSubsystem {
    private final SimpleOutput output;
    private DcMotor top;
    private DcMotor bottom;
    private Servo camServo;
    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION = 1120;
    private static final double WHEEL_RADIUS_INCHES = 0.955;
    private static final double CAM_OPEN_POSITION = 160.0 / 180;
    private static final double CAM_CLOSED_POSITION = 0;
    private static final double LATCH_OPEN_POSITION = .9;
    private static final double LATCH_CLOSED_POSITION = .4;

    private HardwareMap hardwareMap;
    private DigitalChannel topLimit;
    private DigitalChannel bottomLimit;
    private Servo liftLatch;

    public LiftArm(HardwareMap hardwareMap, SimpleOutput output) {
        this.output = output;
        this.hardwareMap=hardwareMap;
    }

    public void init() {
        top = hardwareMap.get(DcMotor.class, Names.LIFT_TOP);
        bottom = hardwareMap.get(DcMotor.class, Names.LIFT_BOTTOM);
        camServo = hardwareMap.get(Servo.class, Names.CAM_SERVO);
        liftLatch = hardwareMap.get(Servo.class, Names.LIFT_LATCH);

        top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topLimit = hardwareMap.get(DigitalChannel.class, Names.TOP_LIMIT);
        topLimit.setMode(DigitalChannel.Mode.INPUT);

        bottomLimit = hardwareMap.get(DigitalChannel.class, Names.BOTTOM_LIMIT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);
        doSleep(100);
    }

    public void prepareToUnlatch() {
        output.write("Lift Arm", "Closing hook...");
        camServo.setPosition(CAM_CLOSED_POSITION);

        output.write("Lift Arm", "Running to bottom limit");
        runToBottomLimit(true, -0.5);
        output.write("Lift Arm", "At bottom limit");

        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double liftPower = -0.5;
        bottom.setPower(liftPower);

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        top.setPower(liftPower);

        liftLatch.setPosition(LATCH_OPEN_POSITION);
        for(int i = 0; i < 5; i++) {
            doSleep(200);
            top.setPower(0);
            doSleep(50);
            top.setPower(liftPower);
        }

        int bottomCurrentPosition = bottom.getCurrentPosition();
        int topCurrentPosition = top.getCurrentPosition();

        bottom.setTargetPosition(bottomCurrentPosition);
        top.setTargetPosition(topCurrentPosition);

        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void landRobot() {
         turnWheels(getStepsToTurn(14.5), bottom);
//        turnWheels(getStepsToTurn(2.5), top);
        camServo.setPosition(CAM_OPEN_POSITION);
        doSleep(1000);
    }

    public void retractLandingGear() {
        camServo.setPosition(CAM_CLOSED_POSITION);
        doSleep(100);
        runToBottomLimit(false, -0.5);

        doSleep(100);
    }

    public void extendLandingGear() {
        liftLatch.setPosition(LATCH_OPEN_POSITION);
        doSleep(100);
        camServo.setPosition(CAM_OPEN_POSITION);
        turnWheels(getStepsToTurn(16.0), bottom);
    }

    public void takeOff() {
        // hole the top motor at current position with full power.
        int currentPosition = top.getCurrentPosition();
        top.setTargetPosition(currentPosition);
        top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        top.setPower(1.0);

        camServo.setPosition(CAM_CLOSED_POSITION);
        doSleep(500);
        runToBottomLimit(false, -1.0);
        liftLatch.setPosition(LATCH_CLOSED_POSITION);
        doSleep(500);

        top.setPower(0);
        bottom.setPower(0);
   }

    private void runToBottomLimit(boolean reset, double power) {

        runToSwitch(this.bottomLimit, power, reset);
    }

    private void runToTopLimit() {
        runToSwitch(this.topLimit, 0.5, false);
    }

    private void runToSwitch(DigitalChannel limitSwitch, double power, boolean reset) {
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setPower(power);
        long lastFalseTime = System.currentTimeMillis();
        while(true) {
            boolean state = limitSwitch.getState();
            // false = triggered. true = open
            if(!state) {
                RobotLog.i("switch state = %s", Boolean.toString(state));
                if(System.currentTimeMillis() - lastFalseTime > 100) {
                    break;
                }
            } else {
                lastFalseTime = System.currentTimeMillis();
            }
            doSleep(10);
        }
        if(reset) {
            bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottom.setTargetPosition(0);
        } else {
            int currentPosition = bottom.getCurrentPosition();
            bottom.setTargetPosition(currentPosition);
        }
        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private int getStepsToTurn(double distance) {
        double wheelRadians = distance / WHEEL_RADIUS_INCHES;
        double wheelTurns = wheelRadians / (2 * Math.PI);

        int steps =  (int)Math.round(wheelTurns * ENCODER_STEPS_PER_WHEEL_ROTATION);

        return steps;
    }

    private void turnWheels(int steps, DcMotor selected) {
        selected.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currentPosition = selected.getCurrentPosition();

        selected.setTargetPosition(currentPosition + steps);
        selected.setPower(0.5);

        while(selected.isBusy()) {
            RobotLog.i("Lift position %d", currentPosition);
            doSleep(10);
            currentPosition = selected.getCurrentPosition();
        }
    }

    private void doSleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException ex) {
            throw new RuntimeException(ex);
        }
    }

    public void lower() {
        runToTopLimit();
        turnWheels(getStepsToTurn(-20),top);
        camServo.setPosition(1);
    }
}
