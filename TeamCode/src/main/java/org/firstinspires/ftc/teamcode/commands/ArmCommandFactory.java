package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;

public class ArmCommandFactory {

    public static Command createSourceDrive(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
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
        );
    }


    public static Command createSourceGroundJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> clawRoll.Upright()),
                new InstantCommand(() -> clawPitch.setAngle(0)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm1, 0),
                                new SetArmAngleCommand(arm2, 112)
                        )
                )
        );
    }

    public static Command createSourceLowJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SetArmAngleCommand(arm1, 55),
                new SetArmAngleCommand(arm2, 120),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(()-> clawPitch.setAngle(65))
                )
        );
    }

    public static Command createSourceMediumJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
                new InstantCommand(()-> clawRoll.Upright()),
                new SetArmAngleCommand(arm1, 95),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new SetArmAngleCommand(arm2, 85),
                        new InstantCommand(()-> clawPitch.setAngle(60), clawPitch)
                )
        );
    }

    public static Command createSourceHighFrontJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
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
        );
    }

    public static Command createSourceHighBackJunction(ClawRoll clawRoll, ClawPitch clawPitch, Arm arm1, Arm arm2) {
        return new ParallelCommandGroup(
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
        );
    }
}
