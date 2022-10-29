package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw extends SubsystemBase {

    public static double GRAB = 0;
    public static double RELEASE = 0.5;

    private Servo servo;

    public Claw(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "claw");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void Grab() {
        servo.setPosition(GRAB);
    }

    public void Release() {
        servo.setPosition(RELEASE);
    }
}