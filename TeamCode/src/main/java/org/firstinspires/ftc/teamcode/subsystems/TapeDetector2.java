package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TapeDetector2 extends SubsystemBase {


    private final Telemetry telemetry;
    private final DigitalChannel tapeDetector;

    public TapeDetector2(HardwareMap hardwareMap, Telemetry telemetry) {
        tapeDetector = hardwareMap.get(DigitalChannel.class, "tape");
        tapeDetector.setMode(DigitalChannel.Mode.INPUT);

        this.telemetry = telemetry;
    }

    public boolean tapeDetected() {
        return tapeDetector.getState();
    }

    @Override
    public void periodic() {
        telemetry.addData("Tape Detected", tapeDetected());
    }
}