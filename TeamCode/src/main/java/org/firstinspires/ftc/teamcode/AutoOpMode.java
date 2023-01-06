package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.MapSelectCommand;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.google.common.collect.ImmutableMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ArmCommandFactory;
import org.firstinspires.ftc.teamcode.commands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveStrafeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForVisionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPitch;
import org.firstinspires.ftc.teamcode.subsystems.ClawRoll;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "OLD AUTO")

public class AutoOpMode extends CommandOpMode {

    private Drive drive;
    OpenCvWebcam webcam;
    private Arm arm1;
    private Arm arm2;
    private Claw claw;
    private ClawPitch clawPitch;
    private ClawRoll clawRoll;


    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive=new Drive(hardwareMap,telemetry);
        arm1 = new Arm(hardwareMap, telemetry, "Arm1", "arm1Pot", "fEncoder", 16,1, 180, DcMotorSimple.Direction.REVERSE, Arm.ARM1_PID, 1, -1);
        arm2 = new Arm(hardwareMap, telemetry, "Arm2","arm2Pot", "lEncoder", 90, 1, 155, DcMotorSimple.Direction.FORWARD, Arm.ARM2_PID, 0.75, -0.75);
        claw = new Claw(hardwareMap);
        clawPitch = new ClawPitch(hardwareMap, telemetry);
        clawRoll = new ClawRoll(hardwareMap);
        ArmCommandFactory.createDriveModeFromFront(clawRoll, clawPitch, arm1, arm2).schedule();
        claw.Grab();

        //clawPitch.setAngle(0);
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
        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
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

        WaitForVisionCommand waitForVisionCommand = new WaitForVisionCommand(pl);
        schedule(new SequentialCommandGroup(
            new WaitUntilCommand(()-> isStarted()),
            // 0. scan marker, up to 5 sec
            waitForVisionCommand.withTimeout(5000),

            new DriveForwardCommand(telemetry, drive, 1, 0.5),

            new DriveStrafeCommand(telemetry, drive, 300, 0.5),
            new MapSelectCommand<>(
                    ImmutableMap.of(
                            VisionPipeline.MarkerPlacement.LOCATION_1, new DriveStrafeCommand(telemetry, drive, -1500, 0.5),
                            VisionPipeline.MarkerPlacement.LOCATION_2, new PrintCommand("Location 2"),
                            VisionPipeline.MarkerPlacement.LOCATION_3, new DriveStrafeCommand(telemetry, drive, 1500,0.5),
                            VisionPipeline.MarkerPlacement.UNKNOWN, new PrintCommand("Location unknown")
                    ),
                    () -> waitForVisionCommand.getPlacement()
            ),
            new DriveForwardCommand(telemetry, drive, -1, 0.5),
            new DriveForwardCommand(telemetry, drive, 36, 0.5),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new DriveForwardCommand(telemetry, drive, 12, 0.5),
                    new DriveForwardCommand(telemetry, drive, -12, 0.5)
                ),
                new PrintCommand("false"),
                () -> waitForVisionCommand.getPlacement() == VisionPipeline.MarkerPlacement.LOCATION_2
            ),
            ArmCommandFactory.createPickupCone1(clawRoll, clawPitch, arm1, arm2)
        ));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
