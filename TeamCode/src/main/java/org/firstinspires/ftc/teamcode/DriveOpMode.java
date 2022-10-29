package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commands.SetArmAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;
    private Arm arm1;
    private Arm arm2;
    private Claw claw;
    private ClawPitch clawPitch;
    private ClawRoll clawRoll;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap, telemetry);
        arm1 = new Arm(hardwareMap, telemetry, "Arm1", "arm1Pot", "fEncoder", 16,1, 180, DcMotorSimple.Direction.REVERSE, Arm.ARM1_PID, 1, -1);
        arm2 = new Arm(hardwareMap, telemetry, "Arm2","arm2Pot", "lEncoder", 90, 1, 155, DcMotorSimple.Direction.FORWARD, Arm.ARM2_PID, 0.75, -0.75);
        claw = new Claw(hardwareMap);
        clawPitch = new ClawPitch(hardwareMap);
        clawRoll = new ClawRoll(hardwareMap);

        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

        // Driver 1
        {
            GamepadEx driver = new GamepadEx(gamepad1);

            driver.getGamepadButton(GamepadKeys.Button.A)
                    .whileHeld(new InstantCommand(()-> claw.Grab(), claw))
                    .whenReleased(new InstantCommand(() -> claw.Release(), claw));

            driver.getGamepadButton(GamepadKeys.Button.B)
                    .whileHeld(new InstantCommand(()-> clawRoll.UpsideDown(), clawRoll))
                    .whenReleased(new InstantCommand(() -> clawRoll.Upright(), clawRoll));

            driver.getGamepadButton(GamepadKeys.Button.X)
                    .whileHeld(new InstantCommand(()-> clawPitch.setAngle(45), clawPitch))
                    .whenReleased(new InstantCommand(() -> clawPitch.setAngle(0), clawPitch));

        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);

            driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(new ParallelCommandGroup(
                    new SetArmAngleCommand(arm1, 1),
                    new SequentialCommandGroup(
                        new WaitCommand(50),
                        new SetArmAngleCommand(arm2, 160)
                    ),
                    new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(60))
                    )
                ));

            driver2.getGamepadButton(GamepadKeys.Button.A).whileHeld(new ParallelCommandGroup(
                    new SetArmAngleCommand(arm1, 0),
                    new SetArmAngleCommand(arm2, 100)
            ));

            driver2.getGamepadButton(GamepadKeys.Button.B).whileHeld(new ParallelCommandGroup(
                    new SetArmAngleCommand(arm1, 50),
                    new SetArmAngleCommand(arm2, 110),
                    new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(60))
                    )
            ));

            driver2.getGamepadButton(GamepadKeys.Button.X).whileHeld(new ParallelCommandGroup(
                    new SetArmAngleCommand(arm1, 95),
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 85)
                    )
            ));

            driver2.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ParallelCommandGroup(
                    new SetArmAngleCommand(arm1, 160),
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 10)
                    )
            ));
        }
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
