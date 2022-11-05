package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.ArmCommandFactory;
import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commands.GrabConeCommand;
import org.firstinspires.ftc.teamcode.commands.SetArmAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.HashMap;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;
    private Arm arm1;
    private Arm arm2;
    private Claw claw;
    private ClawPitch clawPitch;
    private ClawRoll clawRoll;

    private int armScoreState;
    private int armPickUpState;

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
                .toggleWhenPressed(new GrabConeCommand(claw));

            driver.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new InstantCommand(()-> clawRoll.UpsideDown(), clawRoll))
                .whenReleased(new InstantCommand(() -> clawRoll.Upright(), clawRoll));

        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);

            /*pickup from floor
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
            */
            //
            // score select
            //

            driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).whenActive(new SequentialCommandGroup(
                new InstantCommand(()-> {
                    if(armScoreState < 5) {
                        armScoreState++;
                    }
                }),
                new WaitCommand(100),
                new ScheduleCommand(new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(1, ArmCommandFactory.createSourceGroundJunction(clawRoll,  clawPitch, arm1, arm2));
                            put(2, ArmCommandFactory.createSourceLowJunction(clawRoll,  clawPitch, arm1, arm2));
                            put(3, ArmCommandFactory.createSourceMediumJunction(clawRoll,  clawPitch, arm1, arm2));
                            put(4, ArmCommandFactory.createSourceHighFrontJunction(clawRoll,  clawPitch, arm1, arm2));
                            put(5, ArmCommandFactory.createSourceHighBackJunction(clawRoll,  clawPitch, arm1, arm2));
                        }},
                        ()-> armScoreState
                ))
            ));

            driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).whenActive(new SequentialCommandGroup(
                    new InstantCommand(()-> {
                        if(armScoreState > 1) {
                            armScoreState--;
                        }
                    }),
                    new WaitCommand(100),
                    new ScheduleCommand(new SelectCommand(
                            new HashMap<Object, Command>() {{
                                put(1, ArmCommandFactory.createSourceGroundJunction(clawRoll,  clawPitch, arm1, arm2));
                                put(2, ArmCommandFactory.createSourceLowJunction(clawRoll,  clawPitch, arm1, arm2));
                                put(3, ArmCommandFactory.createSourceMediumJunction(clawRoll,  clawPitch, arm1, arm2));
                                put(4, ArmCommandFactory.createSourceHighFrontJunction(clawRoll,  clawPitch, arm1, arm2));
                                put(5, ArmCommandFactory.createSourceHighBackJunction(clawRoll,  clawPitch, arm1, arm2));
                            }},
                            ()-> armScoreState
                    ))
            ));

            //
            // pickup select
            //
            driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)).whenActive(new ScheduleCommand(new SequentialCommandGroup(
                    new InstantCommand(()-> {
                        if(armPickUpState < 5) {
                            armPickUpState++;
                        }
                    }),
                    new WaitCommand(100),
                    new SelectCommand(
                            new HashMap<Object, Command>() {{
//                        put(1, ArmCommandFactory.createSourceGroundJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(2, ArmCommandFactory.createSourceLowJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(3, ArmCommandFactory.createSourceMediumJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(4, ArmCommandFactory.createSourceHighFrontJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(5, ArmCommandFactory.createSourceHighBackJunction(clawRoll,  clawPitch, arm1, arm2));
                            }},
                            ()-> armPickUpState
                    )

            )));

            driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)).whenActive(new ScheduleCommand(new SequentialCommandGroup(
                    new InstantCommand(()-> {
                        if(armPickUpState > 1) {
                            armPickUpState--;
                        }
                    }),
                    new WaitCommand(100),
                    new SelectCommand(
                            new HashMap<Object, Command>() {{
//                        put(1, ArmCommandFactory.createSourceGroundJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(2, ArmCommandFactory.createSourceLowJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(3, ArmCommandFactory.createSourceMediumJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(4, ArmCommandFactory.createSourceHighFrontJunction(clawRoll,  clawPitch, arm1, arm2));
//                        put(5, ArmCommandFactory.createSourceHighBackJunction(clawRoll,  clawPitch, arm1, arm2));
                            }},
                            ()-> armPickUpState
                    )

            )));

            //
            // drive mde
            //
            driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).and(driver2.getGamepadButton((GamepadKeys.Button.RIGHT_BUMPER))).negate().whenActive(new SequentialCommandGroup(
                    new InstantCommand(()->{
                        armScoreState = 0;
                        armPickUpState = 0;
                    }),
                    ArmCommandFactory.createSourceDrive(clawRoll, clawPitch, arm1, arm2)
            ));

        }
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("armScoreState", armScoreState);
        telemetry.update();
    }
}
