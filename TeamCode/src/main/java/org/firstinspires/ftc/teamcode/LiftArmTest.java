package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="LAT", group="Linear OpMode")
public class LiftArmTest extends LinearOpMode {
    LiftArm liftArm;
    @Override
    public void runOpMode(){
        liftArm=new LiftArm(hardwareMap);
        waitForStart();
        liftArm.init();
        liftArm.lower();
    }
}
