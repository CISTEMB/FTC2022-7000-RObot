package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/**
 * This is an example of a more complex path to really test the tuning.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Autonomous(group = "drive", name = "Auto V2 Right")
public class AutoV2opMode extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand splineFollower;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        Pose2d startingPosition = new Pose2d(37, -60, Math.toRadians(-0));
        drive.setPoseEstimate(startingPosition);
        Trajectory pushConeTraj = drive.trajectoryBuilder(startingPosition)
                .strafeLeft(50)
                .build();

        schedule(new WaitUntilCommand(this::isStarted).andThen(
                new TrajectoryFollowerCommand(drive, pushConeTraj)
        ));
    }

}