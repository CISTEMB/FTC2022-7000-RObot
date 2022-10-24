package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

@Config
public class Arm extends SubsystemBase {
    public static Config ARM1_CONFIG = buildArm1Config();
    public static Config ARM2_CONFIG = buildArm2Config();

    public static class Config {
        private String name;
        private String potName;
        private String motorName;

        public PIDFCoefficients pidfCoefficients = new PIDFCoefficients();
        public double potOffset;

        public DcMotorSimple.Direction motorDirection;
        public double softLimitMin;
        public double softLimitMax;
        public double tolerance;
    }

    private static Config buildArm1Config() {
        Config config = new Config();
        config.name = "Arm1";

        config.pidfCoefficients = new PIDFCoefficients(0.07, 0, 0, 0.15);
        config.potName = "arm1Pot";
        config.potOffset = 14.5;

        config.motorName = "fEncoder";
        config.motorDirection = DcMotorSimple.Direction.REVERSE;
        config.softLimitMin = 2;
        config.softLimitMax = 180;
        config.tolerance = 2;

        return config;
    }

    private static Config buildArm2Config() {
        Config config = new Config();
        config.name = "Arm2";

        config.pidfCoefficients = new PIDFCoefficients(0, 0, 0, 0);
        config.potName = "arm2Pot";
        config.potOffset = 90;

        config.motorName = "lEncoder";
        config.motorDirection = DcMotorSimple.Direction.FORWARD;
        config.softLimitMin = 2;
        config.softLimitMax = 150;
        config.tolerance = 2;

        return config;
    }

    private final Config config;
    private final DoubleSupplier feedforwardAngleOffset;
    private final Telemetry t;
    private final AnalogInput pot;
    private final InterpLUT angleLookup = new InterpLUT();

    private final DcMotor pivotMotor;
    private final PIDController controller;
    private double setPoint;
    private boolean pidEnabled;
    private double openLoopPower;

    private ArmState armState;

    public Arm(HardwareMap hardwareMap, Telemetry t, Config config, ArmState armState) {
        this(hardwareMap, t, config, () -> 0, armState);
    }

    public Arm(HardwareMap hardwareMap, Telemetry t, Config config, DoubleSupplier feedforwardAngleOffset, ArmState armState) {
        this.t = t;
        this.config = config;
        this.feedforwardAngleOffset = feedforwardAngleOffset;
        this.armState = armState;
        pot = hardwareMap.get(AnalogInput.class, config.potName);
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

        pivotMotor = hardwareMap.get(DcMotor.class, config.motorName);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setDirection(config.motorDirection);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(config.pidfCoefficients.p, config.pidfCoefficients.i, config.pidfCoefficients.d);
        controller.setTolerance(config.tolerance);
    }

    public double getAngle() {
        final double offset = config.potOffset; //Arm is straight down

        double potentiometerAngle = angleLookup.get(pot.getVoltage());

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

    public boolean atSetPoint() {
        if (!pidEnabled) {
            return false;
        }

        return controller.atSetPoint();
    }

    @Override
    public void periodic(){
        // This should be a interface or something or make a ArmStateCallback and register with ARmState for which arm this is
        if (config.name.equals("Arm1")) {
            armState.setJoint1Angle(Rotation2d.fromDegrees(getAngle()));
        } else if (config.name.equals("Arm2")) {
            armState.setJoint2Angle(Rotation2d.fromDegrees(getAngle()));
        }
        
        controller.setPID(config.pidfCoefficients.p, config.pidfCoefficients.i, config.pidfCoefficients.d);
        controller.setTolerance(config.tolerance);

        double currentAngle = getAngle();
        double feedForwardAngle = currentAngle + feedforwardAngleOffset.getAsDouble();
        t.addData(config.name+"PIDFeedForwardAngle",feedForwardAngle);

        double output;
        if (pidEnabled) {
            t.addData(config.name + "PIDSetPoint", setPoint);

            // Calculate PID output
             output = controller.calculate(
                     currentAngle, setPoint
            );
            t.addData(config.name + "PIDOutput", output);

            // Add feed forward
            double feedForward = Math.sin(currentAngle) * config.pidfCoefficients.f;
            t.addData(config.name+"PIDFeedForwardOutput",feedForward);
            output += feedForward;

            t.addData(config.name + "PIDError", controller.getPositionError());
        } else {
            controller.reset();
            output = openLoopPower;
        }


        if ((output < 0) && (getAngle() <= config.softLimitMin)) {
            output = 0;
        }

        if ((output > 0) && (getAngle() >= config.softLimitMax)) {
            output = 0;
        }

        // Special Arm2 Code
        if (config.name.equals("Arm2")) {
            boolean arm2FrameCollision = armState.detectArm2FrameCollision();
            t.addData(config.name + "FrameCollision", arm2FrameCollision);
            if (output < 0 && arm2FrameCollision) {
                output = 0;
            }
        }

        pivotMotor.setPower(output);

        //telemetry
        t.addData(config.name + "MotorOutput", output);
        t.addData(config.name + "PIDEnabled", pidEnabled);
        t.addData(config.name + "Angle", getAngle());
        t.addData(config.name + "PotVoltage", pot.getVoltage());
    }
}
