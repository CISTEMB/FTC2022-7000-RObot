package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;

public class ArmCommandFactory {

    public static Command createDriveModeFromFront(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SetArmAngleCommand(arm2, 160),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> arm2.getAngle() > 145),
                        new SetArmAngleCommand(arm1, 1)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(20))
                )
        );
    }

    public static Command createDriveModeFromFar(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SequentialCommandGroup(
                        new SetArmAngleCommand(arm2, 160).interruptOn(() -> arm2.getAngle() > 155),
                        new SetArmAngleCommand(arm2, 160)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> arm2.getAngle() > 100),
                        new SetArmAngleCommand(arm1, 1)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(20))
                )
        );
    }

    public static Command createDriveModeFromHighRear(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> arm1.getAngle() < 115),
                        new InstantCommand(()-> clawRoll.Upright())
                ),
                new SetArmAngleCommand(arm1, 1),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> arm1.getAngle() < 155),
                        new SetArmAngleCommand(arm2, 160)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(0))
                )
        );
    }

    public static Command createScoreGroundJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> clawRoll.Upright()),
                new InstantCommand(() -> clawPitch.setAngle(5)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 10),
                                new SetArmAngleCommand(arm2, 95)
                        )
                )
        );
    }

    public static Command createScoreLowJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SetArmAngleCommand(arm1, 40),
                new SetArmAngleCommand(arm2, 125),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(70))
                )
        );
    }

    public static Command createScoreMediumJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SetArmAngleCommand(arm1, 90),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 100),
                        new InstantCommand(()-> clawPitch.setAngle(95))
                )
        );
    }

    public static Command createScoreHighFrontJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SetArmAngleCommand(arm1, 160),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 10)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(90))
                )
        );
    }

    public static Command createScoreHighBackJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new SetArmAngleCommand(arm1, 175),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 25)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(()-> arm1.getAngle() > 145),
                        new InstantCommand(()-> clawRoll.UpsideDown()),
                        new InstantCommand(()-> clawPitch.setAngle(85))

                )
        );
    }

    public static Command createPickupConeFar(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SequentialCommandGroup(
                        new InstantCommand(()-> clawPitch.setAngle(0)),
                        new WaitCommand(250),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 50),
                                new SetArmAngleCommand(arm2, 27.5)
                        )
                )
        );
    }

    public static Command createPickupConeFarReady(Claw claw, ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new RunCommand(()-> claw.BigRelease()),
                new InstantCommand(()-> clawRoll.Upright()),
                new InstantCommand(()-> clawPitch.setAngle(90)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 50),
                                new SetArmAngleCommand(arm2, 45)
                        )
                )
        );
    }

    public static Command createPickupCone1(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new InstantCommand(()-> clawPitch.setAngle(0+15)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 0),
                                new SetArmAngleCommand(arm2, 100)
                        )
                )
        );
    }

    public static Command createPickupCone2(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new InstantCommand(()-> clawPitch.setAngle(0+15)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 15),
                                new SetArmAngleCommand(arm2, 85)
                        )
                )
        );
    }

    public static Command createPickupCone3(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new InstantCommand(()-> clawPitch.setAngle(0+15)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 15),
                                new SetArmAngleCommand(arm2, 90)
                        )
                )
        );
    }

    public static Command createPickupCone4(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new InstantCommand(()-> clawPitch.setAngle(20+15)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 15),
                                new SetArmAngleCommand(arm2, 95)
                        )
                )
        );
    }

    public static Command createPickupCone5(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new InstantCommand(()-> clawPitch.setAngle(30+15)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 20),
                                new SetArmAngleCommand(arm2, 95)
                        )
                )
        );
    }

    public static Command createPickupCone6(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new InstantCommand(()-> clawPitch.setAngle(45+15)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 16),
                                new SetArmAngleCommand(arm2, 143)
                        )
                )
        );
    }

    public static Command createPickupConeSideways(Claw claw, ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright(), clawRoll),
                new InstantCommand(()-> claw.Release(), claw),
                new InstantCommand(()-> clawPitch.setAngle(90), clawPitch),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 40),
                                new SetArmAngleCommand(arm2, 52)
                        )
                )
        );
    }

}
