package org.firstinspires.ftc.teamcode.DadOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Concept: Dad1_5_april_alig", group = "Concept")
//@TeleOp(name = "Concept: AprilTag", group = "Concept"
public class Dad1_5_april_align extends OpMode{
    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    double gear = 1.0;


    //.........April Tags............................
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean detectionFlag = false;
    private boolean runAlignFlag = false;
    // .........

    @Override
    public void init() {
        board.init(hardwareMap);
        //initialArmRotation = board.getArmMotorRotations();

        //..........April Tags..........
        initAprilTag();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //..........
    }
    @Override
    public void loop() {
        double forwardSpeed = gear * -gamepad1.right_stick_y;
        double sideSpeed = gear * gamepad1.right_stick_x;
        double rotateSpeed = gear * gamepad1.left_stick_x;
        double armSpeed = -gamepad2.left_stick_y;
        double millimeters = Math.PI * 80 * board.getMotorRotations();

//............April Tags........
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        } else if (gamepad1.dpad_left) {
            detectionFlag = true;
        } else if (gamepad1.dpad_right){
            runAlignFlag = true;
        }


        if (detectionFlag){

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            telemetry.addData("# AprilTags Detected", currentDetections.size());
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
            if (runAlignFlag){
                telemetry.addLine("'xxx");
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 2){
                        telemetry.addLine("id2 XYZ");
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        sideSpeed = .05*detection.ftcPose.x;
                        if (Math.abs(detection.ftcPose.x) < 1.0){
                            forwardSpeed = .005*detection.ftcPose.y;
                            if (detection.ftcPose.y < 10.0){
                                if (Math.abs(detection.ftcPose.x) > 0.2){
                                    sideSpeed = .05*detection.ftcPose.x;
                                }
                                else {
                                    runAlignFlag = false;
                                }
                            }
                        }
                    }
                }
        }

        }
//......................................



        if(gamepad1.right_bumper) {
            gear += .01;
            if (gear > 3) {
                gear =3;
            }
        }
        if(gamepad1.left_bumper){
            gear -= .01;
            if (gear < 0.001) {
                gear = 0.001;
            }
        }

        if ((Math.abs(forwardSpeed) > Math.abs(sideSpeed) && (Math.abs(forwardSpeed) > Math.abs(rotateSpeed)))){
            board.setForwardSpeed(forwardSpeed);
        }
        else if ((Math.abs(sideSpeed) > Math.abs(rotateSpeed) && (Math.abs(forwardSpeed) < Math.abs(sideSpeed)))) {
            board.setSideMotorSpeed(sideSpeed);
        }
        else {
            board.setRotateSpeed(rotateSpeed);
        }
        if (gamepad1.a){
            board.launchDrone();
        }

        board.setArmSpeed(armSpeed);

        if(gamepad2.a){
            //board.setServoDown();
            //board.setServoDown();
            //board.setRotationDirectionFORWARD();
            board.setClawRotation(0.5);
        }
        if(gamepad2.y){
            //board.setServoUp();
            //board.setRotationDirectionREVERSE();
            board.setClawRotation(30);
        }
        if(gamepad2.x){
            board.setClaw_1Active();
        }
        if(gamepad2.b){
            board.setClaw_1Inactive();
        }

        telemetry.addData("Arm Speed: ", armSpeed);
        telemetry.addData("Motor speed: ", forwardSpeed);
        telemetry.addData("Motor rotations: ", board.getMotorRotations());
        telemetry.addData("Distance: ", millimeters);
        telemetry.addData("gear: ", gear);
//...........April Tags
//        telemetryAprilTag();
        //..................................

        telemetry.update();
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()
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

        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
