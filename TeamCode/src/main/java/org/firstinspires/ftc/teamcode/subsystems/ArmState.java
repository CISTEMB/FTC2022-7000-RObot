package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

public class ArmState {

    public static final double kHeightJoint1 = 13;
    public static final double kLengthArm1 = 12;
    public static final double kLengthArm2 = 9;

//    public static final Translation2d kRobotToJoint1 = new Translation2d(0, 13);
//    public static final Translation2d kJoint1ToJoint2 = new Translation2d(0, -12);
//    public static final Translation2d kJoint2ToJoint3 = new Translation2d(0, -9);

//    private final Arm arm1;
//    private final Arm arm2;
//
//    public ArmState(Arm arm1, Arm arm2) {
//        this.arm1 = arm1;
//        this.arm2 = arm2;
//    }

    private Rotation2d joint1Angle = Rotation2d.fromDegrees(0);;
    public void setJoint1Angle(Rotation2d joint1Angle) {
        this.joint1Angle = joint1Angle;
    }

    private Rotation2d joint2Angle = Rotation2d.fromDegrees(30);;
    public void setJoint2Angle(Rotation2d joint2Angle) {
        this.joint2Angle = joint2Angle;
    }

    public Rotation2d getJoint1Angle() {
//        return Rotation2d.fromDegrees(arm1.getAngle());
//        return Rotation2d.fromDegrees(38);
        return joint1Angle;
    }

    public Rotation2d getJoint2Angle() {
//        return Rotation2d.fromDegrees(arm2.getAngle());
//        return Rotation2d.fromDegrees(30);
        return joint2Angle;
    }

    public Translation2d getRobotToJoint1() {
        return new Translation2d(0, kHeightJoint1);
    }

    public Translation2d getRobotToJoint2() {
        double deltaX = kLengthArm1 * getJoint1Angle().getSin();
        double deltaY = -kLengthArm1 * getJoint1Angle().getCos();
        return getRobotToJoint1().plus(new Translation2d(deltaX, deltaY));
    }

    public Translation2d getRobotToJoint3() {
        Rotation2d angle = Rotation2d.fromDegrees(180).minus(getJoint1Angle()).minus(getJoint2Angle());
        double deltaX = kLengthArm2 * angle.getSin();
        double deltaY = kLengthArm2 * angle.getCos();

        return getRobotToJoint2().plus(new Translation2d(deltaX, deltaY));
    }

    public boolean detectArm2FrameCollision() {
        // https://stackoverflow.com/questions/99353/how-to-test-if-a-line-segment-intersects-an-axis-aligned-rectange-in-2d

        // TODO need to create multiple regions
        // - Robot Frame
        // - Floor

        // Point 1
        System.out.println("Joint2 "+ getRobotToJoint2().toString());
        double x1 = getRobotToJoint2().getX();
        double y1 = getRobotToJoint2().getY();

        // Point 2
        System.out.println("Joint3 "+ getRobotToJoint3().toString());
        double x2 = getRobotToJoint3().getX();
        double y2 = getRobotToJoint3().getY();

        // Check if all four corners of the rectangle are on the same side of the line. The implicit equation for a line through p1 and p2 is:
        int onTheLine = 0;
        int aboveTheLine = 0;
        int belowTheLine = 0;

        //Robot Frame           X,  Y
        // Joint 2 sits around (0.1, 1.0), and the joint 3 hovers around 1.75 inches when laying flat
        // So the collision region has been moved to front half of the robot out of the way of joint 1.
        // This is a hack...
        double topLeft[] =     {5,  1.75};
        double topRight[] =    {10, 1.75};
        double bottomRight[] = {10, -2.5};
        double bottomLeft[] =  {5, -2.5};

        double rectPointsX[] = {topLeft[0], topRight[0], bottomRight[0], bottomLeft[0]};
        double rectPointsY[] = {topLeft[1], topRight[1], bottomRight[1], bottomLeft[1]};

        for (int i = 0; i < 4; i++) {
            double x = rectPointsX[i];
            double y = rectPointsY[i];

            double result = (y2-y1)*x + (x1-x2)*y + (x2*y1-x1*y2);

            System.out.println(result);

            if (result > 0) {
                aboveTheLine++;
            } else if(result < 0) {
                belowTheLine++;
            } else {
                onTheLine++;
            }
        }

        // Substitute all four corners into F(x y). If they're all negative or all positive, there is no intersection.
        // If some are positive and some negative, go to step B

        boolean allPositive = aboveTheLine == 4;
        boolean allNegative = belowTheLine == 4;
        if (allNegative || allPositive) {
            System.out.println("no intersection");
            return false;
        }

        //B. Project the endpoint onto the x axis, and check if the segment's shadow intersects the polygon's shadow. Repeat on the y axis:
        if (x1 > topRight[0] && x2 > topRight[0]) {
            //no intersection (line is to right of rectangle).
            System.out.println("no intersection (line is to right of rectangle)");
            return false;
        } else if (x1 < bottomLeft[0] && x2 < bottomLeft[0]) {
            //no intersection (line is to left of rectangle).
            System.out.println("no intersection (line is to left of rectangle)");
            return false;
        } else if (y1 > topRight[1] && y2 > topRight[1]) {
            // no intersection (line is above rectangle).
            System.out.println("no intersection (line is above rectangle)");
            return false;
        } else if (y1 < bottomLeft[1] && y2 < bottomLeft[1]) {
            // no intersection (line is below rectangle).
            System.out.println("no intersection (line is below rectangle)");
            return false;
        } else {
            // there is an intersection
            System.out.println("there is an intersection");
            return true;
        }
    }
}
