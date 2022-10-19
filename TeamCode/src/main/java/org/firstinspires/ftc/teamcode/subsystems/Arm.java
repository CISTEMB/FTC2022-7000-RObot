package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.util.MathUtil;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {

    private Telemetry t;
    private AnalogInput pot;
    private final InterpLUT angleLookup = new InterpLUT();




    public Arm(HardwareMap hardwareMap, Telemetry t) {
        this.t = t;
        pot = hardwareMap.get(AnalogInput.class, "arm1Pot");
        angleLookup.add(-1,0);
        angleLookup.add(0, 0);
        angleLookup.add(0.108, 15);
        angleLookup.add(0.26, 30);
        angleLookup.add(0.388, 45);
        angleLookup.add(0.51, 60);
        angleLookup.add(0.62, 75);
        angleLookup.add(0.735, 90);
        angleLookup.add(0.84, 105);
        angleLookup.add(0.949, 120);
        angleLookup.add(1.065, 135);
        angleLookup.add(1.194, 150);
        angleLookup.add(1.313, 165);
        angleLookup.add(1.483, 180);
        angleLookup.add(1.666, 195);
        angleLookup.add(1.881, 210);
        angleLookup.add(2.144, 225);
        angleLookup.add(2.472, 240);
        angleLookup.add(2.864, 255);
        angleLookup.add(3.33, 270);
        angleLookup.add(4,270);
        angleLookup.createLUT();
    }

    public double getAngle() {
        final double offset = 0; //Arm is straight down

        final double potentiometerAngle = angleLookup.get(pot.getVoltage());

        return potentiometerAngle + offset;
    }

    @Override
    public void periodic() {
     t.addData("arm1Pot", getAngle());
     t.addData("arm1PotVoltage", pot.getVoltage());
     t.update();

    }
}
