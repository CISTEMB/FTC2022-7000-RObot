package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.MapSelectCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.google.common.collect.ImmutableMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ArmCommandFactory;
import org.firstinspires.ftc.teamcode.commands.WaitForVisionCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This is an example of a more complex path to really test the tuning.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Autonomous(group = "drive", name = "Auto V3 Right")
public class AutoV3RightOpMode extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand splineFollower;
    OpenCvWebcam webcam;
    private Arm arm1;
    private Arm arm2;
    private Claw claw;
    private ClawPitch clawPitch;
    private ClawRoll clawRoll;
    private DigitalChannel tapeDetector;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        arm1 = new Arm(hardwareMap, telemetry, "Arm1", "arm1Pot", "fEncoder", 16,1, 180, DcMotorSimple.Direction.REVERSE, Arm.ARM1_PID, 1, -1);
        arm2 = new Arm(hardwareMap, telemetry, "Arm2","arm2Pot", "lEncoder", 90, 1, 155, DcMotorSimple.Direction.FORWARD, Arm.ARM2_PID, 0.75, -0.75);
        claw = new Claw(hardwareMap);
        clawPitch = new ClawPitch(hardwareMap, telemetry);
        clawRoll = new ClawRoll(hardwareMap);
//        ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2).schedule();
        claw.Grab();
        tapeDetector = hardwareMap.get(DigitalChannel.class,"tape");
        tapeDetector.setMode(DigitalChannel.Mode.INPUT);
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        VisionPipeline pl = new VisionPipeline(hardwareMap, telemetry);
        webcam.setPipeline(pl);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startingPosition);

        Trajectory strafeRight = drive.trajectoryBuilder(startingPosition)
                .strafeRight(4.125)
                .build();

        Trajectory toTape = drive.trajectoryBuilder(strafeRight.end())
                .forward(30)
                .build();
        Pose2d tapePositon = new Pose2d(0,0, Math.toRadians(0));

        Trajectory pushConeTraj = drive.trajectoryBuilder(tapePositon)
                .forward(26.625)
                .build();

        Pose2d scorePosition = new Pose2d(27.5, 0, Math.toRadians(-90));

        Trajectory pickUpCone = drive.trajectoryBuilder(scorePosition)
                .splineTo(new Vector2d(24.25, -23.5), Math.toRadians(-90))
                .build();

        Trajectory goToScore = drive.trajectoryBuilder(pickUpCone.end(), true)
                .splineTo(scorePosition.vec(), Math.toRadians(90))
                .build();

        Trajectory parkLeftTraj = drive.trajectoryBuilder(pushConeTraj.end())
                .strafeLeft(23)
                .build();

        Trajectory parkRightTraj = drive.trajectoryBuilder(pushConeTraj.end())
                .strafeRight(23)
                .build();


        WaitForVisionCommand waitForVisionCommand = new WaitForVisionCommand(pl);
        schedule(new SequentialCommandGroup(
                new WaitUntilCommand(()->isStarted()),
                //waitForVisionCommand.withTimeout(5000),
                new ScheduleCommand(ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2)),
                // Goto the right
                new TrajectoryFollowerCommand(drive, strafeRight),

                // Drive over tape
                new TrajectoryFollowerCommand(drive, toTape).interruptOn(
                        ()->tapeDetector.getState()
                ),
                new InstantCommand(()->drive.setPoseEstimate(tapePositon)),
                new RunCommand(()->drive.updatePoseEstimate()).withTimeout(1000),

                //move to scoring position
                new TrajectoryFollowerCommand(drive, pushConeTraj),
                new TurnCommand(drive, Math.toRadians(-31.0)),

                //score cone
                new ScheduleCommand(ArmCommandFactory.createScoreMidBackJunction(clawRoll, clawPitch, arm1, arm2)),
                new WaitUntilCommand(()-> arm2.getAngle() > 90 && arm1.getAngle() > 169),
                new WaitCommand(1000),
                new InstantCommand(()->claw.Release()),
                new WaitCommand(1000),
                new ScheduleCommand(ArmCommandFactory.createDriveModeFromMidRear(clawRoll, clawPitch, arm1, arm2)),
                new WaitCommand(1000),

                //turn twords stack
                new TurnCommand(drive, Math.toRadians(-94+31)),
                new ScheduleCommand(ArmCommandFactory.createPickupCone5(clawRoll, clawPitch, arm1, arm2)),
                new WaitCommand(1000),
                new InstantCommand(()->claw.Release()),
                new TrajectoryFollowerCommand(drive, pickUpCone),
                new WaitCommand(500),
                new InstantCommand(()->claw.Grab()),
                new WaitCommand(500),
                new ScheduleCommand(new SequentialCommandGroup(
                        ArmCommandFactory.createPickupCone6(clawRoll, clawPitch, arm1, arm2).withTimeout(250),
                        new ScheduleCommand(ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2))
                )),
                new WaitCommand(500),
                new TrajectoryFollowerCommand(drive, goToScore),
                new TurnCommand(drive, Math.toRadians(90-31.0)),

                //Score second cone
                new ScheduleCommand(ArmCommandFactory.createScoreMidBackJunction(clawRoll, clawPitch, arm1, arm2)),
                new WaitUntilCommand(()-> arm2.getAngle() > 90 && arm1.getAngle() > 169),
                new WaitCommand(1000),
                new InstantCommand(()->claw.Release()),
                new WaitCommand(1000),
                new ScheduleCommand(ArmCommandFactory.createDriveModeFromMidRear(clawRoll, clawPitch, arm1, arm2)),
                new WaitCommand(1000),
                new TurnCommand(drive, Math.toRadians(31.0))

//                //To-do: Score cones from the cone stack during autonomous
//
//                //Park in the correct space
//                new TurnCommand(drive, Math.toRadians(31.0)),
//                new MapSelectCommand<>(
//                    ImmutableMap.of(
//                            VisionPipeline.MarkerPlacement.LOCATION_1, new TrajectoryFollowerCommand(drive,parkLeftTraj),
//                            VisionPipeline.MarkerPlacement.LOCATION_2, new PrintCommand("Location 2"),
//                            VisionPipeline.MarkerPlacement.LOCATION_3, new TrajectoryFollowerCommand(drive,parkRightTraj),
//                            VisionPipeline.MarkerPlacement.UNKNOWN, new PrintCommand("Location unknown")
//                    ),
//                    () -> waitForVisionCommand.getPlacement()
//                ),
//            new InstantCommand(() -> new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(drive.getPoseEstimate()).back(12).build()).schedule())
        ));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Drive Pos X", drive.getPoseEstimate().getX());
        telemetry.addData("Drive Pos Y", drive.getPoseEstimate().getY());
        telemetry.addData("Drive Pos Heading",Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.update();
    }

}