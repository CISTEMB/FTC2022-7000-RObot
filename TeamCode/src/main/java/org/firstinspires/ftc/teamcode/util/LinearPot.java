package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class LinearPot {

    private final AnalogInput analogInput;
    private final InterpLUT angleLookup = new InterpLUT();;

    public LinearPot(AnalogInput analogInput) {
        this.analogInput = analogInput;

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
        return angleLookup.get(analogInput.getVoltage());
    }

}
