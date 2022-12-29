package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.ArmCommandFactory;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This is an example of a more complex path to really test the tuning.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Autonomous(group = "drive", name = "Auto V2 Right")
public class AutoV2opMode extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand splineFollower;
    OpenCvWebcam webcam;
    private Arm arm1;
    private Arm arm2;
    private Claw claw;
    private ClawPitch clawPitch;
    private ClawRoll clawRoll;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        arm1 = new Arm(hardwareMap, telemetry, "Arm1", "arm1Pot", "fEncoder", 16,1, 180, DcMotorSimple.Direction.REVERSE, Arm.ARM1_PID, 1, -1);
        arm2 = new Arm(hardwareMap, telemetry, "Arm2","arm2Pot", "lEncoder", 90, 1, 155, DcMotorSimple.Direction.FORWARD, Arm.ARM2_PID, 0.75, -0.75);
        claw = new Claw(hardwareMap);
        clawPitch = new ClawPitch(hardwareMap);
        clawRoll = new ClawRoll(hardwareMap);
//        ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2).schedule();
        claw.Grab();

        Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startingPosition);

        Trajectory strafeRight = drive.trajectoryBuilder(startingPosition)
                .strafeRight(4.125)
                .build();

        Trajectory pushConeTraj = drive.trajectoryBuilder(strafeRight.end())
                .forward(54)
                .build();

        schedule(new SequentialCommandGroup(
                new WaitUntilCommand(()->isStarted()),
                new ScheduleCommand(ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2)),
//                new InstantCommand(()->telemetry.addData("autoState", "strafe")),
                new TrajectoryFollowerCommand(drive, strafeRight),
//                new InstantCommand(()->telemetry.addData("autoState", "push")),
                new TrajectoryFollowerCommand(drive, pushConeTraj),
//                new InstantCommand(()->telemetry.addData("autoState", "turn")),
                new TurnCommand(drive, Math.toRadians(-33.33)),
//                new InstantCommand(()->telemetry.addData("autoState", "turnDone"))
//                new InstantCommand(()->telemetry.addData("autoState", "score")),
                new ScheduleCommand(ArmCommandFactory.createScoreMidBackJunction(clawRoll, clawPitch, arm1, arm2)),
                new WaitUntilCommand(()-> arm2.getAngle() > 90 && arm1.getAngle() > 169),
                new InstantCommand(()->claw.Release()),
                new WaitCommand(1000),
                new ScheduleCommand(ArmCommandFactory.createDriveModeFromMidRear(clawRoll, clawPitch, arm1, arm2))
        ));

    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }

}