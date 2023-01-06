package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ClawPitch extends SubsystemBase {

    private ServoEx servo;
    private Telemetry telemetry;

    public ClawPitch(HardwareMap hardwareMap, Telemetry telemetry){
        servo = new SimpleServo(hardwareMap, "clawPitch", 0, 270);
        this.telemetry = telemetry;
        servo.setInverted(false);
    }

    public void setAngle(double angle){
       servo.turnToAngle(angle);
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Pitch", servo.getAngle());
    }
}