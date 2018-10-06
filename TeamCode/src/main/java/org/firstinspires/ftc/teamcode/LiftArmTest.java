package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

@TeleOp(name="Lift Arm Test", group="iterative op mode")
public class LiftArmTest extends BasicOpMode_Iterative {
    private DcMotor top;
    private DcMotor bottom;
    @Override
    public void init() {
        top=hardwareMap.get(DcMotor.class, "top");
        bottom=hardwareMap.get(DcMotor.class,"bottom");
    }

    @Override
    public void loop() {
        double power;
        power = gamepad1.right_stick_y;
        top.setPower(power);
        bottom.setPower(power);
    }
}
