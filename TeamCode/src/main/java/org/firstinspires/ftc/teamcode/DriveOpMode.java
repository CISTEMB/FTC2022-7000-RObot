package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;

    @Override
    public void initialize(){
        drive = new Drive(hardwareMap, telemetry);

        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

        // Driver 1
        {
            GamepadEx driver = new GamepadEx(gamepad1);



        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);



        }
    }
}
