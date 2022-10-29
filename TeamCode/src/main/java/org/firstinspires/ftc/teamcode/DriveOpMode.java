package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;
    private Arm arm1;
    private Arm arm2;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap, telemetry);
        arm1 = new Arm(hardwareMap, telemetry, "Arm1", "arm1Pot", "fEncoder", 16,1, 180, DcMotorSimple.Direction.REVERSE, Arm.ARM1_PID);
        arm2 = new Arm(hardwareMap, telemetry, "Arm2","arm2Pot", "lEncoder", 90, 1, 155, DcMotorSimple.Direction.FORWARD, Arm.ARM2_PID);

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

            driver.getGamepadButton(GamepadKeys.Button.A)
                    .whileHeld(new InstantCommand(()-> arm1.setAngle(90), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

            driver.getGamepadButton(GamepadKeys.Button.X)
                    .whileHeld(new InstantCommand(()-> arm1.setAngle(45), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

            driver.getGamepadButton(GamepadKeys.Button.Y)
                    .whileHeld(new InstantCommand(()-> arm1.setAngle(135), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));


        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);

            driver2.getGamepadButton(GamepadKeys.Button.A)
                    .whileHeld(new InstantCommand(()-> arm2.setAngle(0), arm2))
                    .whenReleased(new InstantCommand(() -> arm2.setPower(0), arm2));

            driver2.getGamepadButton(GamepadKeys.Button.X)
                    .whileHeld(new InstantCommand(()-> arm2.setAngle(45), arm2))
                    .whenReleased(new InstantCommand(() -> arm2.setPower(0), arm2));

            driver2.getGamepadButton(GamepadKeys.Button.Y)
                    .whileHeld(new InstantCommand(()-> arm2.setAngle(90), arm2))
                    .whenReleased(new InstantCommand(() -> arm2.setPower(0), arm2));

            driver2.getGamepadButton(GamepadKeys.Button.B)
                    .whileHeld(new InstantCommand(()-> arm2.setAngle(145), arm2))
                    .whenReleased(new InstantCommand(() -> arm2.setPower(0), arm2));



        }
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
