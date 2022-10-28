package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {

    private Servo servo;

    public Claw(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "claw");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void Grab() {
        servo.setPosition(0);
    }

    public void Release() {
        servo.setPosition(1);
    }
}