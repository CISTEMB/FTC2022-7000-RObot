package org.firstinspires.ftc.teamcode.commands.roadrunner;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.TrajectoryProvider;

public class TrajectoryFollowerCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final TrajectoryProvider trajectoryProvider;
    private boolean stop = true;

    public TrajectoryFollowerCommand doNotStop() {
        stop=false;
        return this;
    }

    public TrajectoryFollowerCommand(MecanumDriveSubsystem drive, Trajectory trajectory) {
        this(drive, ()->trajectory);
    }

    public TrajectoryFollowerCommand(MecanumDriveSubsystem drive, TrajectoryProvider trajectoryProvider) {
        this.drive = drive;
        this.trajectoryProvider = trajectoryProvider;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectory(trajectoryProvider.get());
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        drive.breakFollowing();
        if (interrupted && stop) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}