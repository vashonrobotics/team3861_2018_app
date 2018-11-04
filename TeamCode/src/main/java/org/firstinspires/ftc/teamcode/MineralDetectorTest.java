package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mineral Detector test", group="Iterative Opmode")
public class MineralDetectorTest extends OpMode {
    OpenCVMineralDetector mineralDetector;
    GoldDetector goldDetector;
    @Override
    public void init() {
        mineralDetector=new OpenCVMineralDetector(hardwareMap);
        mineralDetector.init();
    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {
        mineralDetector.isGold();
        telemetry.addData("is found:",mineralDetector.isGold());
    }

    @Override
    public void stop(){

    }
}
