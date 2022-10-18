package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.util.MathUtil;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {

    private Telemetry t;
    private AnalogInput pot;


    public Arm(HardwareMap hardwareMap, Telemetry t) {
        this.t = t;

        pot = hardwareMap.get(AnalogInput.class, "arm1Pot");
    }

    public double getAngle() {

        final double voltageIn = 0;
        final double angleIn = 0;

        final double voltageOut = 3.3;
        final double angleOut = 270;
        //final double potentiometerAngle = (((angleOut - angleIn) * (pot.getVoltage() - voltageIn)) / (voltageOut - voltageIn)) + angleIn;
        //final double potentiometerAngle = Range.scale(pot.getVoltage(),0,3.3,0,270);
        final double offset = 0; //Arm is straight down

        final double potentiometerAngle =

        return potentiometerAngle + offset;
    }

    @Override
    public void periodic() {
     t.addData("arm1Pot", getAngle());
     t.addData("arm1PotVoltage", pot.getVoltage());












     t.update();

    }
}
