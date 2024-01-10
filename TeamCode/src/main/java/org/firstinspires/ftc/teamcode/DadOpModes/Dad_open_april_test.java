package org.firstinspires.ftc.teamcode.DadOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Concept: Dad1_open_april_test", group = "Concept")
//@TeleOp(name = "Concept: AprilTag", group = "Concept"
public class Dad_open_april_test extends OpMode{
//    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    double gear = 1.0;


    //.........April Tags............................
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;


    private VisionPortal visionPortal;
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortalopenCV;
    private boolean detectionFlag = false;
    private boolean runAlignFlag = false;
    private boolean flagCreateVisionPortal = true;
    // .........

    @Override
    public void init() {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                        hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        visionProcessor = new FirstVisionProcessor();
//        visionPortalopenCV = VisionPortal.easyCreateWithDefaults(
//                hardwareMap.get(WebcamName.class, "Webcam 2"), visionProcessor);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //..........
    }
    @Override
    public void loop() {

//............April Tags........

        if (gamepad1.dpad_up) {
            if (flagCreateVisionPortal==true) {
                visionPortalopenCV = VisionPortal.easyCreateWithDefaults(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
                flagCreateVisionPortal = false;
            }
        }else if (gamepad1.dpad_down) {
            visionPortalopenCV.close();
            flagCreateVisionPortal=true;
        } else if (gamepad1.dpad_right){
            if (flagCreateVisionPortal==true) {
                visionPortalopenCV = VisionPortal.easyCreateWithDefaults(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
                flagCreateVisionPortal=false;
          }
        } else if (gamepad1.dpad_left) {
            visionPortal.stopStreaming();
        } else{}//pass}

        telemetry.update();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }
}