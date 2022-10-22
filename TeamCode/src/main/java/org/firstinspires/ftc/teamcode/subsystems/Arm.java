package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {

    private final Telemetry t;
    private final AnalogInput pot;
    private final InterpLUT angleLookup = new InterpLUT();

    private final DcMotor pivotMotor;
    private final PIDFController controller = new PIDFController(0, 0, 0, 0);
    private double setPoint;
    private boolean pidEnabled;
    private double openLoopPower;


    public Arm(HardwareMap hardwareMap, Telemetry t) {
        this.t = t;
        pot = hardwareMap.get(AnalogInput.class, "arm1Pot");
        angleLookup.add(-1,0);
        angleLookup.add(0, 0);
        angleLookup.add(0.108, 15);
        angleLookup.add(0.26, 30);
        angleLookup.add(0.388, 45);
        angleLookup.add(0.51, 60);
        angleLookup.add(0.62, 75);
        angleLookup.add(0.735, 90);
        angleLookup.add(0.84, 105);
        angleLookup.add(0.949, 120);
        angleLookup.add(1.065, 135);
        angleLookup.add(1.194, 150);
        angleLookup.add(1.313, 165);
        angleLookup.add(1.483, 180);
        angleLookup.add(1.666, 195);
        angleLookup.add(1.881, 210);
        angleLookup.add(2.144, 225);
        angleLookup.add(2.472, 240);
        angleLookup.add(2.864, 255);
        angleLookup.add(3.33, 270);
        angleLookup.add(4,270);
        angleLookup.createLUT();

        pivotMotor = hardwareMap.get(DcMotor.class, "fEncoder");
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getAngle() {
        final double offset = 16; //Arm is straight down

        final double potentiometerAngle = angleLookup.get(pot.getVoltage());

        return potentiometerAngle - offset;
    }

    public void setPower(double power){
        openLoopPower = power;
        setPIDEnabled(false);
    }

    public void setAngle(double angle){
        setPoint = angle;
        setPIDEnabled(true);
    }

    public void setPIDEnabled(boolean enabled){
        pidEnabled = enabled;
    }

    @Override
    public void periodic(){
        double output;
        if (pidEnabled) {
             output = controller.calculate(
                    getAngle(), setPoint
            );

        } else {
            controller.reset();
            output = openLoopPower;
        }

        if ((output < 0) && (getAngle() <= 0)) {
            output = 0;
        }

        if ((output > 0) && (getAngle() >= 250)) {
            output = 0;
        }

        pivotMotor.setPower(output);

        //telemetry
        t.addData("arm1Pot", getAngle());
        t.addData("arm1PotVoltage", pot.getVoltage());
        t.update();

    }
}
