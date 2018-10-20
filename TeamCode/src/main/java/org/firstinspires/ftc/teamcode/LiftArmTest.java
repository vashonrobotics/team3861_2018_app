package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LiftArmTest extends LinearOpMode {
    LiftArm liftArm;

    public void runOpMode(){
        liftArm=new LiftArm(hardwareMap);
        liftArm.init();
        // liftArm.raise();
    }
}
