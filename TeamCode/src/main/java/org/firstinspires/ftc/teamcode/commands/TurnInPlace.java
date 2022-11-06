package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class TurnInPlace extends CommandBase {

    private Telemetry t;
    private Drive drive = null;
    private double goal;
    private boolean isFinished;

    public TurnInPlace(Drive drive, double goal, Telemetry t) {

        this.drive = drive;
        addRequirements(drive);

        this.goal = goal;

        this.t = t;

    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {

        //get our heading
        double heading = drive.getHeading();
        //calculate error: e = goal - heading
        double error = goal - heading;
        //figure out how much to turn :(
        double kP = -0.010;
        double output = error * kP;
        double floor = 0.25;
        if (Math.abs(output) < floor) {
            output = Math.copySign(floor, output);
        }
        //turn :)
        drive.arcadeDrive(0,output, 0, false);

        double kTolerance = 1;
        isFinished = Math.abs(error) < kTolerance;
        t.addData("Heading: ", heading);
        t.addData("Error: ", error);
        t.addData("Output: ", output);

//        debug.putNumber("Heading: ", heading);
//        debug.putNumber("Error: ", error);
//        debug.putNumber("Output: ", output);
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
