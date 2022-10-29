package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ClawPitch extends SubsystemBase {

    private ServoEx servo;

    public ClawPitch(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "clawPitch", 0, 270);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
       servo.turnToAngle(angle);
    }
}