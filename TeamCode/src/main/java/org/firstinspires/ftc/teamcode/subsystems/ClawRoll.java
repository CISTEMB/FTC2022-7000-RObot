package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawRoll extends SubsystemBase {

    public static double UPRIGHT_POSITION = 0.87;
    public static double UPSIDE_DOWN = 0.15;

    private Servo servo;

    public ClawRoll(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "clawRoll");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void Upright() {
        servo.setPosition(UPRIGHT_POSITION);
    }

    public void UpsideDown() {
        servo.setPosition(UPSIDE_DOWN);
    }
}