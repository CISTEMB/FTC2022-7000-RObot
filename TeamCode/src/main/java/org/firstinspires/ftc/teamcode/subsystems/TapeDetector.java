package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TapeDetector extends SubsystemBase {

    public static float COLOR_SENSOR_GAIN = (float) 6.3;

    private final Telemetry telemetry;
    private final NormalizedColorSensor colorSensor;
    final float[] hsvValues = new float[3];

    public TapeDetector(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        this.telemetry = telemetry;
    }


    public double getHue() {
        return hsvValues[0];
    }
    public double getSaturation() {
        return hsvValues[1];
    }
    public double getValue() {
        return hsvValues[2];
    }

    @Override
    public void periodic() {
        colorSensor.setGain(COLOR_SENSOR_GAIN);
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsvValues);

        telemetry.addData("Tape Hue", getHue());
        telemetry.addData("Tape Saturation", getSaturation());
        telemetry.addData("Tape Value", getValue());
    }
}