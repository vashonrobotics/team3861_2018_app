package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Collector
{
    private final HardwareMap hardwareMap;
    private final SimpleOutput simpleOutput;
    private DcMotor drive;
    private DcMotor axel;
    private DcMotor extender;

    //Want to lower and spin collector.

    public Collector(HardwareMap hardwareMap, SimpleOutput simpleOutput) {
        this.hardwareMap = hardwareMap;
        this.simpleOutput = simpleOutput;
    }

    public void init(){

        drive = hardwareMap.get(DcMotor.class, Names.COLLECTOR_DRIVE);
        axel = hardwareMap.get(DcMotor.class, Names.COLLECTOR_UP_DOWN);
        extender = hardwareMap.get(DcMotor.class, "Extendor");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        axel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        axel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void lowerAndWait(){
        lowerCollector();
        while(axel.isBusy()){
            try {
                Thread.sleep(10);
            }catch (InterruptedException ex) {
            }
        }
    }
    public void lowerCollector(){
        axel.setPower(-0.75);
    }
    public void raiseCollector(){
        axel.setPower(1);
    }

    public void stopRaiseLower() {
        axel.setPower(0);
    }
    public void extend(){
        extender.setPower(.75);
    }

    public void retract(){
        extender.setPower(-.75);
    }
    public void stopRetract(){
        extender.setPower(0);
    }

    public void suck(){
        drive.setPower(.75);
    }
    public void blow(){
        drive.setPower(-.75);
    }
    public void stop(){
        drive.setPower(0);
    }

}
