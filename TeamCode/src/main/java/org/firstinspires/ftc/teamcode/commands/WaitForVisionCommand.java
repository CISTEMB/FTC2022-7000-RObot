package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VisionPipeline;

public class WaitForVisionCommand extends CommandBase {
    private VisionPipeline pl;
    private VisionPipeline.MarkerPlacement placement;

    public VisionPipeline.MarkerPlacement getPlacement() {
        return placement;
    }

    public WaitForVisionCommand(VisionPipeline pl){
       this.pl=pl;
    }

    @Override
    public void initialize() {
        placement = VisionPipeline.MarkerPlacement.UNKNOWN;
    }

    @Override
    public void execute() {
        placement = pl.getMarkerPlacement();
    }

    @Override
    public boolean isFinished() {
        return placement != VisionPipeline.MarkerPlacement.UNKNOWN;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
