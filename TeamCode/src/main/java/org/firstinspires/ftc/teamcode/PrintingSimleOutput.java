package org.firstinspires.ftc.teamcode;

public class PrintingSimleOutput implements SimpleOutput {
    @Override
    public void write(String caption, String message) {
        System.out.println(caption + ":");
        System.out.println(message);
    }
}
