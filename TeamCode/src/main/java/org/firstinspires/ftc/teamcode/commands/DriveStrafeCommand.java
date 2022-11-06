package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DriveStrafeCommand extends CommandBase {
    private Drive drive;
    private int goalCounts;
    private double power;
    private Telemetry t;
    private boolean isFinished;

    public DriveStrafeCommand(Telemetry t, Drive drive, int goalCounts, double power) {
        this.drive = drive;
        this.goalCounts = goalCounts;
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
        drive.runToPositionStrafe(goalCounts, power);
//        double error = goal - drive.getLeftFrontMotorInches();
        double kTolerance = 0.25;
        //isFinished = Math.abs(error) < kTolerance;


        int[] counts = drive.getAllEncoderCounts();
        double avgError = 0;
        avgError += goalCounts - counts[0]; // Left Front
        avgError += goalCounts - counts[3]; // Right Back
        avgError /= 2;

        isFinished = Math.abs(avgError) < 10;

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
