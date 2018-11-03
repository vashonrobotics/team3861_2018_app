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
    //Want to lower and spin collector.

    public Collector(HardwareMap hardwareMap, SimpleOutput simpleOutput) {
        this.hardwareMap = hardwareMap;
        this.simpleOutput = simpleOutput;
    }

    public void init(){

        drive = hardwareMap.get(DcMotor.class, Names.COLLECTOR_DRIVE);
        axel = hardwareMap.get(DcMotor.class, Names.COLLECTOR_UP_DOWN);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        axel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lowerCollector(){
        axel.setTargetPosition(725);
        axel.setPower(0.75);
    }
    public void raiseCollector(){
        axel.setTargetPosition(0);
        axel.setPower(0.75);
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
