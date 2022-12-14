package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        StandardTrackingWheelLocalizer encoders = new  StandardTrackingWheelLocalizer(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){


            telemetry.addData("Front", encoders.getWheelPositions().get(2));
            telemetry.addData("Right", encoders.getWheelPositions().get(1));
            telemetry.addData("Left", encoders.getWheelPositions().get(0));
            telemetry.update();

        }
    }
}
