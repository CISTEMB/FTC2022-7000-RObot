package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DriveWithGamepadCommand extends CommandBase {

    private Gamepad gamepad;
    private Drive drive;
    private Arm arm1;

    public DriveWithGamepadCommand(Gamepad gamepad, Drive drive, Arm arm1) {
        this.gamepad = gamepad;
        this.drive = drive;
        this.arm1 = arm1;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double forward = gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;
        forward = -forward;

        if (gamepad.left_bumper || arm1.getAngle().getDegrees() > 20){
            forward *= 0.75;
            turn *= 0.35;
            strafe *= 0.75;
        }

        if (Math.abs(strafe) <= Math.abs(forward)){
          strafe = 0;
        }else {
            strafe = Math.copySign(Math.abs(strafe) - Math.abs(forward), strafe);
        }

        drive.arcadeDrive(forward, turn, strafe, true);



    }

    @Override
    public boolean isFinished() { return false;}


    @Override
    public void end(boolean interrupted) {

    }

}
