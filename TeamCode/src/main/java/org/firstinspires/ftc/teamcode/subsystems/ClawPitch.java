package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawPitch extends SubsystemBase {

    private Servo servo;

    public ClawPitch(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "clawPitch");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(double angle){
       servo.setPosition(angle);
    }
}