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


    /**
     * Created by FTC on 9/23/2017.
     * updated by Caz
     */

    @TeleOp(name = "Mec TeleOp", group = "Vashon 3861")
    public class TeleOpMode extends OpMode{
        private static final double MARKER_DUMPED_POS = .15;
        private static final double MARKER_RAISED_POS = .66;
        private Collector collector;
        private DcMotor lift;
        private LiftArm liftArm;
        private double motorSpeedMultiplier = 1.0;
        private ArrayList baseMotorArray = new ArrayList();
        private int previousBaseMotorPos = -1;
        private TelemetrySimpleOutput simpleOutput;
        private Servo markerServo;
//    boolean pressedA = false;

        @Override
        public void init() {
            // base motor init
            baseMotorArray.add(hardwareMap.dcMotor.get(Names.LEFT_FRONT));
            baseMotorArray.add(hardwareMap.dcMotor.get(Names.RIGHT_FRONT));
            baseMotorArray.add(hardwareMap.dcMotor.get(Names.LEFT_REAR));
            baseMotorArray.add(hardwareMap.dcMotor.get(Names.RIGHT_REAR));
            ((DcMotor)baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
            ((DcMotor)baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);

            markerServo = hardwareMap.get(Servo.class, Names.MARKER_SERVO);
            markerServo.setPosition(MARKER_RAISED_POS);
            liftArm=new LiftArm(hardwareMap,simpleOutput);
            liftArm.init();
            collector=new Collector(hardwareMap,simpleOutput);
            collector.init();
        }

        @Override
        public void loop() {
            boolean doLengthen = gamepad2.a;
            boolean doShorten = gamepad2.b;
            boolean doSuck = gamepad2.dpad_down;
            boolean doBlow = gamepad2.dpad_up;
            boolean doLower = gamepad2.left_bumper;
            boolean doRaise = gamepad2.right_bumper;
            boolean doDump = gamepad1.x;

            if(doDump){
                markerServo.setPosition(MARKER_DUMPED_POS);
            }else{
                markerServo.setPosition(MARKER_RAISED_POS);
            }

            if(doBlow){
                collector.blow();
            } else if(doSuck){
                collector.suck();
            } else {
                collector.stop();
            }

            if(doLengthen){
                collector.extend();
            } else if(doShorten){
                collector.retract();
            } else {
                collector.stopRetract();
            }

            if(doRaise){
                collector.raiseCollector();
            } else if(doLower){
                collector.lowerCollector();
            } else {
                collector.stopRaiseLower();
            }


            boolean doExtend = gamepad1.dpad_up;
            boolean doRetract = gamepad1.dpad_down;
            if(doExtend) {
                liftArm.extendLandingGear();
            } else if(doRetract) {
                liftArm.takeOff();
            }

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


            if (gamepad1.right_trigger >= 0.5) {
                motorSpeedMultiplier = 0.4;
            }else {
                motorSpeedMultiplier = 1.0;
            }
            DriveTrainBoogaloo.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    (-(double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier, true);

            telemetry.update();
        }
    }


