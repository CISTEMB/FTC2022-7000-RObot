package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DriveForwardCommand extends CommandBase {
    private Drive drive;
    private double goal;
    private double power;
    private Telemetry t;
    private boolean isFinished;

    public DriveForwardCommand(Telemetry t, Drive drive, double goal, double power) {
        this.drive = drive;
        this.goal = goal;
        this.power = power;
        this.t = t;
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        isFinished = false;
        drive.stop();
    }

    @Override
    public void execute() {
        drive.runToPosition(goal, power);
//        double error = goal - drive.getLeftFrontMotorInches();
        double kTolerance = 0.25;
        //isFinished = Math.abs(error) < kTolerance;


        double goalCounts = drive.getCountFromInches(goal);
        int[] counts = drive.getAllEncoderCounts();
        double avgError = 0;
        avgError += goalCounts - counts[0]; // Left Front
        avgError += goalCounts - counts[2]; // Right Front
        avgError /= 2;

        isFinished = Math.abs(avgError) < drive.getCountFromInches(0.1);

        t.addData("Average Error: ", avgError);
        t.addData("Is Finished: ", isFinished);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
