package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commands.GrabConeCommand;
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
                new DriveWithGamepadCommand(gamepad1, drive, arm1)
        );

        // Driver 1
        {
            GamepadEx driver = new GamepadEx(gamepad1);

            driver.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(new GrabConeCommand(claw));

            driver.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new InstantCommand(()-> clawRoll.UpsideDown(), clawRoll))
                .whenReleased(new InstantCommand(() -> clawRoll.Upright(), clawRoll));

        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);

            //drive
            driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ParallelCommandGroup(
                    new InstantCommand(()-> clawRoll.Upright()),
                    new SetArmAngleCommand(arm2, 160),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> arm2.getAngle().getDegrees() > 145),
                        new SetArmAngleCommand(arm1, 1)
                    ),
                    new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(60))
                    )
                ));

            //pickup from floor
            driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ParallelCommandGroup(
                    new InstantCommand(()-> clawRoll.Upright()),
                    new InstantCommand(()-> clawPitch.setAngle(0)),
                    new SequentialCommandGroup(
                            new WaitCommand(100),
                            new ParallelCommandGroup(
                                    new SetArmAngleCommand(arm1, 0),
                                    new SetArmAngleCommand(arm2, 100)
                            )
                    )
            ));
            //ground junction
            driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ParallelCommandGroup(
                    new InstantCommand(()-> clawRoll.Upright()),
                    new InstantCommand(()-> clawPitch.setAngle(0)),
                    new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 0),
                                new SetArmAngleCommand(arm2, 112)
                        )
                    )
            ));

            //low
            driver2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ParallelCommandGroup(
                    new InstantCommand(()-> clawRoll.Upright()),
                    new SetArmAngleCommand(arm1, 55),
                    new SetArmAngleCommand(arm2, 120),
                    new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(65))
                    )
            ));

            //medium
            driver2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ParallelCommandGroup(
                    new InstantCommand(()-> clawRoll.Upright()),
                    new SetArmAngleCommand(arm1, 95),
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 85),
                        new InstantCommand(()-> clawPitch.setAngle(60), clawPitch)
                    )
            ));

            //high front
            driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ParallelCommandGroup(
                    new InstantCommand(()-> clawRoll.Upright()),
                    new SetArmAngleCommand(arm1, 160),
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 10)
                    ),
                    new SequentialCommandGroup(
                            new WaitCommand(100),
                            new InstantCommand(()-> clawPitch.setAngle(60))
                    )
            ));

            //high back
            driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ParallelCommandGroup(
                    new SetArmAngleCommand(arm1, 167),
                    new SequentialCommandGroup(
                            new WaitCommand(300),
                            new SetArmAngleCommand(arm2, 30)
                    ),
                    new SequentialCommandGroup(
                            new WaitUntilCommand(()-> arm1.getAngle().getDegrees() > 145),
                            new InstantCommand(()-> clawRoll.UpsideDown()),
                            new InstantCommand(()-> clawPitch.setAngle(60))

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
