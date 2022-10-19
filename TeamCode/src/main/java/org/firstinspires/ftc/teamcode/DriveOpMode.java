package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;
    private Arm arm1;

    @Override
    public void initialize(){
        drive = new Drive(hardwareMap, telemetry);
        arm1 = new Arm(hardwareMap, telemetry);

        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

        // Driver 1
        {
            GamepadEx driver = new GamepadEx(gamepad1);
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whileHeld(new InstantCommand(()-> arm1.setPower(1.0), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whileHeld(new InstantCommand(()-> arm1.setPower(-0.2), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);



        }
    }
}
