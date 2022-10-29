package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class WaitForArmAngle extends CommandBase {

    private Arm arm;
    private double angle;

    public WaitForArmAngle(Arm arm, double angle){
        this.arm = arm;
        this.angle = angle;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
