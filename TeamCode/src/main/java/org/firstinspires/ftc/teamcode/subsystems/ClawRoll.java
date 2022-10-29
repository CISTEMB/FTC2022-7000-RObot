package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawRoll extends SubsystemBase {

    private Servo servo;

    public ClawRoll(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "clawRoll");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void Upright() {
        servo.setPosition(0);
    }

    public void UpsideDown() {
        servo.setPosition(1);
    }
}