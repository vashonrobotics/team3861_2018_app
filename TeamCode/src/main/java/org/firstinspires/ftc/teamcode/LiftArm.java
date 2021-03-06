package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;


public class LiftArm implements hardwareSubsystem {
    private final SimpleOutput output;
    public DcMotor bottom;
    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION = 1680;
    private static final double WHEEL_RADIUS_INCHES = 0.25;
    private static final double LATCH_OPEN_POSITION = 0.92;
    private static final double LATCH_CLOSED_POSITION = .4;

    private HardwareMap hardwareMap;
    private DigitalChannel topLimit;
    private DigitalChannel bottomLimit;
    private Servo liftLatch;
    private RevTouchSensor latchSwitch;

    public LiftArm(HardwareMap hardwareMap, SimpleOutput output) {
        this.output = output;
        this.hardwareMap=hardwareMap;
    }

    public void init() {
        bottom = hardwareMap.get(DcMotor.class, Names.LIFT_BOTTOM);
        liftLatch = hardwareMap.get(Servo.class, Names.LIFT_LATCH);
        latchSwitch = hardwareMap.get(RevTouchSensor.class, Names.LATCH_SWITCH);

        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //topLimit = hardwareMap.get(DigitalChannel.class, Names.TOP_LIMIT);
        //topLimit.setMode(DigitalChannel.Mode.INPUT);

        //bottomLimit = hardwareMap.get(DigitalChannel.class, Names.BOTTOM_LIMIT);
        //bottomLimit.setMode(DigitalChannel.Mode.INPUT);
        doSleep(100);
    }

    public void prepareToUnlatch() {
        output.write("Lift Arm", "Closing hook...");
        int bottomCurrentPosition = bottom.getCurrentPosition();

        boolean switchValue = latchSwitch.isPressed();
        while(!switchValue) {
            liftLatch.setPosition(LATCH_OPEN_POSITION);
            int stepsToTurn = getStepsToTurn(-0.125);
            bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottom.setTargetPosition(bottomCurrentPosition + stepsToTurn);
            bottom.setPower(1.0);

            long startTime = System.currentTimeMillis();
            while (bottom.isBusy()
                    && !switchValue
                    && System.currentTimeMillis() - startTime < 100) {
                doSleep(10);
                switchValue = latchSwitch.isPressed();
            }
            switchValue = latchSwitch.isPressed();
            bottomCurrentPosition = bottom.getCurrentPosition();
        }

        RobotLog.i("Exiting unlatch code");
    }

    public void landRobot() {

//        turnWheels(getStepsToTurn(2.5), top);
        turnWheels(getStepsToTurn(8), bottom);
        doSleep(1000);
    }

    public void retractLandingGear() {

        turnWheels(getStepsToTurn(-8.),bottom);
        bottom.setPower(0.0);
        doSleep(100);
    }

    public void extendLandingGear() {
        liftLatch.setPosition(LATCH_OPEN_POSITION);
        doSleep(100);
        turnWheels(getStepsToTurn(-8.5), bottom);
    }

    public void takeOff() {
        // hole the top motor at current position with full power.
        doSleep(1000);
        turnWheels(getStepsToTurn(8),bottom);
        liftLatch.setPosition(LATCH_CLOSED_POSITION);
        doSleep(500);

        bottom.setPower(0);
   }

    //private void runToBottomLimit(boolean reset, double power) {

      //  runToSwitch(this.bottomLimit, power, reset);
   // }

   // private void runToTopLimit() {
   //     runToSwitch(this.topLimit, 0.5, false);
   // }

//    private void runToSwitch(DigitalChannel limitSwitch, double power, boolean reset) {
//        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bottom.setPower(power);
//        long lastFalseTime = System.currentTimeMillis();
//        while(true) {
//            boolean state = limitSwitch.getState();
//            // false = triggered. true = open
//            if(!state) {
//                RobotLog.i("switch state = %s", Boolean.toString(state));
//                if(System.currentTimeMillis() - lastFalseTime > 100) {
//                    break;
//                }
//            } else {
//                lastFalseTime = System.currentTimeMillis();
//            }
//            doSleep(10);
//        }
//        if(reset) {
//            bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            bottom.setTargetPosition(0);
//        } else {
//            int currentPosition = bottom.getCurrentPosition();
//            bottom.setTargetPosition(currentPosition);
//        }
//        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

   public int getStepsToTurn(double distance) {
        double wheelRadians = distance / WHEEL_RADIUS_INCHES;
        double wheelTurns = wheelRadians / (2 * Math.PI);

        int steps =  (int)Math.round(wheelTurns * ENCODER_STEPS_PER_WHEEL_ROTATION);

        return steps;
    }

    public void turnWheels(int steps, DcMotor selected) {
        selected.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currentPosition = selected.getCurrentPosition();

        liftLatch.setPosition(LATCH_OPEN_POSITION);
        selected.setTargetPosition(currentPosition + steps);
        selected.setPower(0.5);
        int lastPosition = 0;
        int countUnmoved = 0;
        while(selected.isBusy() && countUnmoved < 5) {
            RobotLog.i("Lift position %d", currentPosition);
            doSleep(10);
            lastPosition = currentPosition;
            currentPosition = selected.getCurrentPosition();
            if(Math.abs(currentPosition - lastPosition) < 5) {
                countUnmoved++;
            } else {
                countUnmoved = 0;
            }
        }
    }

    private void doSleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException ex) {
            throw new RuntimeException(ex);
        }
    }

}
