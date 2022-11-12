package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionPipeline extends OpenCvPipeline {

    public enum MarkerPlacement {
        LOCATION_1,
        LOCATION_2,
        LOCATION_3,
        UNKNOWN
    }

    private MarkerPlacement markerPlacement = MarkerPlacement.UNKNOWN;

    public MarkerPlacement getMarkerPlacement() {
        return markerPlacement;
    }

    private DigitalChannel redLED;
    private DigitalChannel greenLED;
    private RevBlinkinLedDriver leds;

    /*
     * Cache
     */
    private Mat output = new Mat();

    private Telemetry telemetry;

    public VisionPipeline(HardwareMap hardwareMap, Telemetry telemetry){
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "Blingin");

        this.telemetry = telemetry;

    }

    @Override
    public Mat processFrame(Mat input) {
//        System.out.println(input.dump());
        // I/System.out: Mat [ 240*320*CV_8UC4, isCont=true, isSubmat=false, nativeObj=0xe870b880, dataAddr=0xce783000 ]
        //System.out.println(input.toString());

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2BGR);

        List<Mat> markerCorners = new ArrayList<>();
        MatOfInt markerIDs = new MatOfInt();

        //
        // Detect Aruco markers
        //
        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_250);
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(input, dict, markerCorners, markerIDs, parameters);

        //
        // Determine which team marker spot it is on (If markers are present)
        //


        markerPlacement = MarkerPlacement.UNKNOWN;

        int[] markerIDArray = new int[0];
        if (!markerCorners.isEmpty()) {
            markerIDArray = markerIDs.toArray();
        }

        for (int i = 0; i < markerCorners.size(); i++) {
            int markerID = markerIDArray[i];


            if (markerID == 1) {
                System.out.println("LOCATION_1");
                markerPlacement=MarkerPlacement.LOCATION_1;
                break;
            } else if (markerID == 2) {
                System.out.println("LOCATION_2");
                markerPlacement=MarkerPlacement.LOCATION_2;
                break;
            } else if (markerID == 23) {
                System.out.println("LOCATION_3");
                markerPlacement=MarkerPlacement.LOCATION_3;
                break;
            } else {
                System.out.println("Ignoring marker " + markerID);
            }


        }

        telemetry.addData("Marker", markerPlacement);

        //
        // Annotate the source image
        //
        input.copyTo(output);

        if (markerPlacement == MarkerPlacement.LOCATION_1) {
            // blue, amber
            redLED.setState(false);
            greenLED.setState(false);
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        } else if (markerPlacement == MarkerPlacement.LOCATION_2) {
            // green
            redLED.setState(false);
            greenLED.setState(true);
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (markerPlacement == MarkerPlacement.LOCATION_3) {
            // red
            greenLED.setState(false);
            redLED.setState(true);
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            greenLED.setState(true);
            redLED.setState(true);
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
        }

        Aruco.drawDetectedMarkers(output, markerCorners, markerIDs);
        return output;
    }

}
