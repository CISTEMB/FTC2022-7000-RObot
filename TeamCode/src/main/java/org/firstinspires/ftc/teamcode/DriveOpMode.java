package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.ArmCommandFactory;
import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commands.GrabConeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.HashMap;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private RevBlinkinLedDriver leds;
    private Drive drive;
    private Arm arm1;
    private Arm arm2;
    private Claw claw;
    private ClawPitch clawPitch;
    private ClawRoll clawRoll;

    private int armScoreState;
    private int armPickUpState;
    private int farPickUpState;

    private boolean wasAtHighRear = false;
    private boolean atHighRear = false;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap, telemetry);
        arm1 = new Arm(hardwareMap, telemetry, "Arm1", "arm1Pot", "fEncoder", 16,1, 180, DcMotorSimple.Direction.REVERSE, Arm.ARM1_PID, 1, -1);
        arm2 = new Arm(hardwareMap, telemetry, "Arm2","arm2Pot", "lEncoder", 90, 1, 155, DcMotorSimple.Direction.FORWARD, Arm.ARM2_PID, 0.75, -0.75);
        claw = new Claw(hardwareMap);
        clawPitch = new ClawPitch(hardwareMap, telemetry);
        clawRoll = new ClawRoll(hardwareMap);
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "Blingin");
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE );

        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive, arm1)
        );

        claw.Grab();

        // Driver 1
        {
            GamepadEx driver = new GamepadEx(gamepad1);

            driver.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(new GrabConeCommand(claw));

            driver.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new InstantCommand(()-> clawRoll.UpsideDownHB(), clawRoll))
                .whenReleased(new InstantCommand(() -> clawRoll.Upright(), clawRoll));

        }

        // Driver2
        {
            GamepadEx driver2 = new GamepadEx(gamepad2);

            //fast high back score
            driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                    new SequentialCommandGroup(
                        new InstantCommand(()->{
                            atHighRear = true;
                            farPickUpState = 0;
                        }),
                        ArmCommandFactory.createScoreHighBackJunction(clawRoll,clawPitch,arm1,arm2)
                    )
            );

            //pickup from floor
            driver2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                    ArmCommandFactory.createPickupConeSideways(claw, clawRoll,clawPitch,arm1,arm2)
            );
            driver2.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                    ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2)
            );

            //pickup from far
//            driver2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                    ArmCommandFactory.createPickupConeFar(clawRoll, clawPitch, arm1, arm2)
//            );
//            driver2.getGamepadButton(GamepadKeys.Button.B).whenReleased(
//                    ArmCommandFactory.createDriveModeFromFar(clawRoll, clawPitch, arm1, arm2)
//            );
//
//            driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                    ArmCommandFactory.createPickupConeFarReady(clawRoll, clawPitch, arm1, arm2)
//            );
//            driver2.getGamepadButton(GamepadKeys.Button.A).whenReleased(
//                    ArmCommandFactory.createDriveModeFromFar(clawRoll, clawPitch, arm1, arm2)
//            );

            driver2.getGamepadButton(GamepadKeys.Button.B).whenActive(new ScheduleCommand(new SequentialCommandGroup(
                    new InstantCommand(()-> {
                        farPickUpState = (farPickUpState + 1) % 2;
                    }),
                    new SelectCommand(
                            new HashMap<Object, Command>() {{
                              put(1, new ScheduleCommand(ArmCommandFactory.createPickupConeFarReady(claw, clawRoll, clawPitch, arm1, arm2)));
                              put(0, new ScheduleCommand(ArmCommandFactory.createPickupConeFar(clawRoll, clawPitch, arm1, arm2)));
                            }},
                            ()-> farPickUpState
                    )
            )));

            //
            // score select
            //

            driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).whenActive(new SequentialCommandGroup(
                new InstantCommand(()-> {
                    if(armScoreState < 4) {
                        armScoreState++;
                    }
                }),
                new WaitCommand(100),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(1, new ScheduleCommand(ArmCommandFactory.createScoreGroundJunction(clawRoll,  clawPitch, arm1, arm2)));
                            put(2, new ScheduleCommand(ArmCommandFactory.createScoreLowJunction(clawRoll,  clawPitch, arm1, arm2)));
                            put(3, new ScheduleCommand(ArmCommandFactory.createScoreMediumJunction(clawRoll,  clawPitch, arm1, arm2)));
                            put(4, new ScheduleCommand(ArmCommandFactory.createScoreHighFrontJunction(clawRoll,  clawPitch, arm1, arm2)));
                        }},
                        ()-> armScoreState
                )
            ));

            driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).whenActive(new SequentialCommandGroup(
                    new InstantCommand(()-> {
                        if(armScoreState > 1) {
                            armScoreState--;
                        }
                    }),
                    new WaitCommand(100),
                            new SelectCommand(
                            new HashMap<Object, Command>() {{
                                put(1, new ScheduleCommand(ArmCommandFactory.createScoreGroundJunction(clawRoll,  clawPitch, arm1, arm2)));
                                put(2, new ScheduleCommand(ArmCommandFactory.createScoreLowJunction(clawRoll,  clawPitch, arm1, arm2)));
                                put(3, new ScheduleCommand(ArmCommandFactory.createScoreMediumJunction(clawRoll,  clawPitch, arm1, arm2)));
                                put(4, new ScheduleCommand(ArmCommandFactory.createScoreHighFrontJunction(clawRoll,  clawPitch, arm1, arm2)));
                            }},
                            ()-> armScoreState
                    )
            ));

            //
            // pickup select
            //
            driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)).whenActive(new ScheduleCommand(new SequentialCommandGroup(
                    new InstantCommand(()-> {
                        if(armPickUpState < 6) {
                            armPickUpState++;
                        }
                    }),
                    new WaitCommand(100),
                    new SelectCommand(
                            new HashMap<Object, Command>() {{
                                put(1, new ScheduleCommand(ArmCommandFactory.createPickupCone1(clawRoll,  clawPitch, arm1, arm2)));
                                put(2, new ScheduleCommand(ArmCommandFactory.createPickupCone2(clawRoll,  clawPitch, arm1, arm2)));
                                put(3, new ScheduleCommand(ArmCommandFactory.createPickupCone3(clawRoll,  clawPitch, arm1, arm2)));
                                put(4, new ScheduleCommand(ArmCommandFactory.createPickupCone4(clawRoll,  clawPitch, arm1, arm2)));
                                put(5, new ScheduleCommand(ArmCommandFactory.createPickupCone5(clawRoll,  clawPitch, arm1, arm2)));
                                put(6, new ScheduleCommand(ArmCommandFactory.createPickupCone6(clawRoll,  clawPitch, arm1, arm2)));
                            }},
                            ()-> armPickUpState
                    )

            )));

            driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)).whenActive(new ScheduleCommand(new SequentialCommandGroup(
                    new InstantCommand(()-> {
                        if(armPickUpState > 1) {
                            armPickUpState--;
                        }

                        if (armPickUpState == 0) {
                            armPickUpState = 1;
                        }
                    }),
                    new WaitCommand(100),
                    new SelectCommand(
                            new HashMap<Object, Command>() {{
                                put(1, new ScheduleCommand(ArmCommandFactory.createPickupCone1(clawRoll,  clawPitch, arm1, arm2)));
                                put(2, new ScheduleCommand(ArmCommandFactory.createPickupCone2(clawRoll,  clawPitch, arm1, arm2)));
                                put(3, new ScheduleCommand(ArmCommandFactory.createPickupCone3(clawRoll,  clawPitch, arm1, arm2)));
                                put(4, new ScheduleCommand(ArmCommandFactory.createPickupCone4(clawRoll,  clawPitch, arm1, arm2)));
                                put(5, new ScheduleCommand(ArmCommandFactory.createPickupCone5(clawRoll,  clawPitch, arm1, arm2)));
                                put(6, new ScheduleCommand(ArmCommandFactory.createPickupCone6(clawRoll,  clawPitch, arm1, arm2)));
                            }},
                            ()-> armPickUpState
                    )

            )));

            //
            // drive mode
            //
            driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).or(driver2.getGamepadButton((GamepadKeys.Button.RIGHT_BUMPER))).negate().whenActive(new SequentialCommandGroup(
                    new ConditionalCommand(
                            ArmCommandFactory.createPickupCone6(clawRoll, clawPitch, arm1, arm2).withTimeout(250),
                            new PrintCommand("false"),
                            () -> armPickUpState >= 4
                    ),
                    new InstantCommand(()->{
                        armScoreState = 0;
                        armPickUpState = 0;
                        farPickUpState = 0;

                        wasAtHighRear = atHighRear;
                        atHighRear = false;
                    }),

                    new ConditionalCommand(
                            ArmCommandFactory.createDriveModeFromHighRear(clawRoll, clawPitch, arm1, arm2),
                            ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2),
                            () -> wasAtHighRear
                    )
            ));

        }
        ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2).schedule();

    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("armScoreState", armScoreState);
        telemetry.addData("armPickUpState", armPickUpState);

        telemetry.update();
    }
}
