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

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;
    private ArmState armState = new ArmState();
    private Arm arm1;
    private Arm arm2;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        drive = new Drive(hardwareMap, telemetry);
        arm1 = new Arm(hardwareMap, telemetry, Arm.ARM1_CONFIG, armState);
        arm2 = new Arm(hardwareMap, telemetry, Arm.ARM2_CONFIG, armState);

        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

        // Driver 1
        {
            GamepadEx driver = new GamepadEx(gamepad1);
//            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                    .whileHeld(new InstantCommand(()-> arm1.setPower(1.0), arm1))
//                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));
//
//            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                    .whileHeld(new InstantCommand(()-> arm1.setPower(-0.2), arm1))
//                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whileHeld(new InstantCommand(()-> arm2.setPower(0.5), arm2))
                    .whenReleased(new InstantCommand(() -> arm2.setPower(0), arm2));

            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whileHeld(new InstantCommand(()-> arm2.setPower(-0.5), arm2))
                    .whenReleased(new InstantCommand(() -> arm2.setPower(0), arm2));

            // Closed loop control
            driver.getGamepadButton(GamepadKeys.Button.A)
                    .whileHeld(new InstantCommand(()-> arm1.setAngle(90), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

            driver.getGamepadButton(GamepadKeys.Button.B)
                    .whileHeld(new InstantCommand(()-> arm1.setAngle(45), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

            driver.getGamepadButton(GamepadKeys.Button.X)
                    .whileHeld(new InstantCommand(()-> arm1.setAngle(135), arm1))
                    .whenReleased(new InstantCommand(() -> arm1.setPower(0), arm1));

            driver.getGamepadButton(GamepadKeys.Button.Y)
                    .whileHeld(new InstantCommand(()-> arm2.setAngle(90), arm2))
                    .whenReleased(new InstantCommand(() -> arm2.setPower(0), arm2));

        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);



        }
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("RobotToJoint2", armState.getRobotToJoint2().toString());
        telemetry.addData("RobotToJoint3", armState.getRobotToJoint3().toString());
        telemetry.update();
    }
}
