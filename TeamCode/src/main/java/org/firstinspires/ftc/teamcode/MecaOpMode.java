package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Range;
import java.util.ArrayList;
import static com.qualcomm.robotcore.util.Range.clip;

public class MecaOpMode {

    /**
     * Created by FTC on 9/23/2017.
     * updated by Caz
     */

    @TeleOp(name = "Vashon 5961 teleop", group = "Vashon 5961")
    public class TeleOpMode extends OpMode{
        private DcMotor lift;
        private double motorSpeedMultiplier = 1.0;
        private ArrayList baseMotorArray = new ArrayList();
        private int liftTargetPos = 0;
        private DcMotor collectorExtender;
        private DcMotor collectorRotator;
        private Servo collectorGrabber;
        private Servo collectorGrabberRotator;
        private Servo markerDropper;
        private Boolean setMode = false;
        private int previousBaseMotorPos = -1;
//    private CRServo stickyArm;
//    boolean pressedA = false;

        @Override
        public void init() {
            // base motor init
            baseMotorArray.add(hardwareMap.dcMotor.get("motorLF"));
            baseMotorArray.add(hardwareMap.dcMotor.get("motorRF"));
            baseMotorArray.add(hardwareMap.dcMotor.get("motorLB"));
            baseMotorArray.add(hardwareMap.dcMotor.get("motorRB"));
            ((DcMotor)baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
            ((DcMotor)baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);
//
//        // lift init
            lift = hardwareMap.dcMotor.get("lift");

            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        stickyArm = hardwareMap.crservo.get("stickyArm");

//
//        markerDropper = hardwareMap.servo.get("dropper");

            // collector init
//        collectorRotator = hardwareMap.dcMotor.get("rotate");
//        collectorRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        collectorExtender = hardwareMap.dcMotor.get("extend");
//        collectorExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        collectorGrabber = hardwareMap.servo.get("grab");
//        collectorGrabberRotator = hardwareMap.servo.get("assistant");
//        for(int i = 0; i < 4; i++){
//            ((DcMotor)baseMotorArray.get(i)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
        }

        @Override
        public void loop() {
//        telemetry.addData("arm power",stickyArm.getPower());
            if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
//            ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
                if (previousBaseMotorPos != -1) {
                    if(previousBaseMotorPos != ((DcMotor) baseMotorArray.get(0)).getCurrentPosition()){
                        telemetry.addData("change in encoder values", ((DcMotor) baseMotorArray.get(0)).getCurrentPosition() - previousBaseMotorPos);
                    }
                }else{
                    previousBaseMotorPos = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
                }
            }
            if(gamepad1.x){
                previousBaseMotorPos = -1;
            }
//        for (int i = 0; i < baseMotorArray.size(); i++) {
//            DcMotor motor = ((DcMotor) baseMotorArray.get(i));
//            telemetry.addData("motor " + i, motor.getCurrentPosition());
//        }
//        telemetry.addData("lift encoder", lift.getCurrentPosition());
//        telemetry.addData("lift Target Pos", liftTargetPos);
//        if (gamepad2.right_stick_x > 0.1 || gamepad2.right_stick_x < -0.1) {
//            liftTargetPos += gamepad2.right_stick_x*2;
//
//            lift.setTargetPosition(liftTargetPos);
//            lift.setPower(1);
//        }

            lift.setPower(gamepad2.right_stick_x);

//        stickyArm.setPower(gamepad2.left_stick_x/2-.4);


            if (gamepad1.right_trigger >= 0.5) {
                motorSpeedMultiplier = 0.4;
            }else {
                motorSpeedMultiplier = 1.0;
            }
            DriveTrainBoogaloo.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    (-(double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier, true);

//
//            //  up is negative
            // collector stuff
//        collectorExtender.setPower(gamepad2.left_stick_x);
//        collectorRotator.setPower(gamepad2.left_stick_y/3);
//        if (gamepad2.right_bumper) {
//            collectorGrabberRotator.setPosition(1);
//        }else{
//            collectorGrabberRotator.setPosition(0);
//        }
//        if (gamepad2.left_bumper){
//            collectorGrabber.setPosition(1);
//        }else{
//            collectorGrabber.setPosition(0.4);
//        }
            telemetry.update();
        }
    }

}
