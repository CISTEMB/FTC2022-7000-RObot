package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class GrabConeCommand extends CommandBase {

    private Claw claw;

    public GrabConeCommand(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        claw.Grab();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        claw.Release();
    }
}
