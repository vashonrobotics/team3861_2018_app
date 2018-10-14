package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;


public class LiftArm implements hardwareSubsystem {
    private DcMotor top;
    private DcMotor bottom;
    private Servo camServo;
    private static final double MAX_POS=1;
    private static final double MIN_POS=0;
    private static final double COUNTS_PER_MOTOR_REV = 4 ;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 40 ;     // This is < 1.0 if geared UP
    private static final double ENCODER_STEPS_PER_WHEEL_ROTATION =
            COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private static final double WHEEL_RADIUS_INCHES = 3.5;
    private HardwareMap hardwareMap;
    public LiftArm(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
    }
    public void init() {
        top=hardwareMap.get(DcMotor.class, "top");
        bottom=hardwareMap.get(DcMotor.class,"bottom");
        camServo=hardwareMap.get(Servo.class,"camServo");
        top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void raise() {
        turnWheels(getStepsToTurn(20),bottom);
        turnWheels(getStepsToTurn(20),top);
        camServo.setPosition(0);
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

    public void lower(){
        turnWheels(getStepsToTurn(-20),bottom);
        turnWheels(getStepsToTurn(-20),top);
        camServo.setPosition(1);
    }
}
