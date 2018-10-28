package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySimpleOutput implements SimpleOutput {
    private final Telemetry telemetry;

    public TelemetrySimpleOutput(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void write(String caption, String message) {
        telemetry.addData(caption, message);
        telemetry.update();
    }
}
