package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Autonomous
//@Disabled
public class DucksAutonomous9_DAS extends OpMode {
    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    double forwardconstant = Math.PI * 75 * 523.875 / 457.2 * 514.35 / 457.2 * 417.5125 / 457.2 * 665 / 635 * 641 / 635 * 638 / 635;
    double rotationConstant = 360 * ((75 * Math.PI) / (533.4 * Math.PI)) * 92 / 90 * 90.7 / 90 * 88.8103 / 90;
    double sideconstant = Math.PI * 75 * 534 / 508 * 510 / 508 * 512 / 508;
    double armconstant = 360 * 30 / 125 * 30 / 125;
    int state;
    int position;
    FirstVisionProcessor.Selected Position;
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    //    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
//    private VisionPortal VisionPortal;

    public double id_1_x_position;
    public double id_1_y_position;
    public double id_2_x_position;
    public double id_2_y_position;
    public double id_3_x_position;
    public double id_3_y_position;
//    public double rotate_degree;
//    public double degree;

    @Override
    public void init() {
        visionProcessor = new FirstVisionProcessor();
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class,
                        "Webcam 1"),
                visionProcessor);
        board.init(hardwareMap);
    }
    @Override
    public void init_loop() {
        telemetry.addData("Identified_loop",
                visionProcessor.getSelection());
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        /*if (state ==0){
            MoveRotateDegrees(90,0.5);
            state = 54;
            */
        if (state == 0) {
            board.setClaw_1Active();
            board.setClaw_2Active();
            board.setClawRotation(0);
            state = 1;
        } else if (state == 1) {
            ducksSleepMilliSec(500);
            state = 2;
        } else if (state == 2) {
            MoveArmDegrees(15, 0.5);
            state = 3;
        } else if (state == 3) {
            board.setClawRotation(0.7);
            state = 4;
        } else if (state == 4) {
            ducksSleepMilliSec(100);
            state = 5;
        } else if (state == 5) {
            MoveSidewaysDistance(75);    //...........................
            state = 6;
        } else if (state == 6) {
            ducksSleepMilliSec(1000);
            state = 7;
        } else if (state == 7) {
            Position = visionProcessor.getSelection();
            if (Position.equals(FirstVisionProcessor.Selected.MIDDLE)) {
                position = 2;
                MoveSidewaysDistance(65);
                centerPlacement();
                state = 8;
            } else if (Position == FirstVisionProcessor.Selected.LEFT) {
                position = 1;
                MoveSidewaysDistance(-75);
                leftPlacement();
                state = 8;
            } else {
                position = 3;
                //MoveSidewaysDistance(-75);
                rightPlacement();
                state = 8;
            }
        } else if (state == 8) {
//            visionPortal.stopStreaming();
            /*visionPortal.close();
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
*/
            //ducksSleepMilliSec(1500);
            state = 9;
        }

        if (state == 9) {
//            VisionPortal.resumeStreaming();
//            initAprilTag();
            //ducksSleepMilliSec(300);
            state = 10;
        } else if (state == 10) {
            //MoveForwardDistance(100, 0.2);
            //ducksSleepMilliSec(200);
            state = 11;
        } else if (state == 11) {
            telemetryAprilTag();
            telemetry.addData("main loop", id_1_x_position);
            if (position == 1) {
                if (Math.abs(id_1_x_position) > 1) {
                    if (id_1_x_position > 0) {
                        board.setSideMotorSpeed(0.1);
                    } else if (id_1_x_position < 0) {
                        board.setSideMotorSpeed(-0.1);
                    }
                } else {
                    board.setSideMotorSpeed(0.0);
                    state = 12;
                }
            } else if (position == 2) {
                if (Math.abs(id_2_x_position) > 1) {
                    if (id_2_x_position > 0) {
                        telemetry.addData("ID 2 x position: ", id_2_x_position);
                        board.setSideMotorSpeed(0.1);
                        telemetry.update();
                    } else if (id_2_x_position < 0) {
                        telemetry.addData("ID 2 x position: ", id_2_x_position);
                        board.setSideMotorSpeed(-0.1);
                        telemetry.update();
                    }
                } else {
                    board.setSideMotorSpeed(0.0);
                    state = 12;
                }
            } else {
                if (Math.abs(id_3_x_position) > 0.3) {
                    if (id_3_x_position > 0) {
                        board.setSideMotorSpeed(0.1);
                    } else if (id_3_x_position < 0) {
                        board.setSideMotorSpeed(-0.1);
                    }
                } else {
                    board.setSideMotorSpeed(0.0);
                    state = 12;
                }
            }

        } else if (state == 12) {
            board.setClawRotation(0.25);
            MoveSidewaysDistance(50);
            state = 13;
        } else if (state == 13) {
            telemetryAprilTag();
            telemetry.addData("main loop", id_1_y_position);
            if (position == 1) {
                if (id_1_y_position > 20) {
                    board.setForwardSpeed(0.2);
                } else {
                    board.setForwardSpeed(0.0);
                    state = 14;
                }
            } else if (position == 2) {
                if (id_2_y_position > 20) {
                    board.setForwardSpeed(0.2);
                } else {
                    board.setForwardSpeed(0.0);
                    state = 14;
                }
            } else {
                if (id_3_y_position > 20) {
                    board.setForwardSpeed(0.2);
                } else {
                    board.setForwardSpeed(0.0);
                    state = 14;
                }
            }

        } else if (state == 14) {
            MoveForwardDistance(65, 0.1);
            state = 15;
        } else if (state == 15) {
            board.setClaw_2Inactive();
            ducksSleepMilliSec(300);
            state = 16;
        } else if (state == 16) {
            MoveForwardDistance(-100, 0.2);
            board.setClawRotation(0.7);
            if (position == 1) {
                MoveSidewaysDistance(-500);
            } else if (position == 2) {
                MoveSidewaysDistance(-680);
            } else {
                MoveSidewaysDistance(500);
            }
            MoveForwardDistance(480, 0.3);
            state = 17;
        }

        telemetry.addData("state = ", state);
        telemetry.update();

    }
    public void MoveRotateDegrees(double degrees, double rotateSpeed) {
        double initialWheelRotation = board.getMotorRotations();
        double deg_tolerance = 0.5;
        double delta_deg;
        delta_deg = degrees - (rotationConstant * (board.getMotorRotations()-initialWheelRotation));
        while (Math.abs(delta_deg) > deg_tolerance ){
            if (delta_deg > 0) {
                board.setRotateSpeed(rotateSpeed);
            } else if (delta_deg < 0){
                board.setRotateSpeed(-rotateSpeed);
            }
            delta_deg= degrees - (rotationConstant * (board.getMotorRotations()-initialWheelRotation));
            if (Math.abs(delta_deg) < deg_tolerance){
                board.setRotateSpeed(0);
                ducksSleepMilliSec(100);
                delta_deg= degrees - (rotationConstant * (board.getMotorRotations()-initialWheelRotation));
                rotateSpeed=0.1;
            }
            telemetry.addData("delta_deg = ", delta_deg);
            telemetry.update();
        }
        board.setRotateSpeed(0);
        telemetry.addData("delta_deg = ", delta_deg);
        telemetry.update();
        ducksSleepMilliSec(5000);
    }
    /*
        public void MoveRotateDegrees(double degrees, double rotateSpeed) {
            double initialWheelRotation = board.getMotorRotations();
            double mm = (rotationConstant * (board.getMotorRotations()-initialWheelRotation));
            if (degrees > 0) {
                while (mm < degrees) {
                    //elevatorheight();
                    board.setRotateSpeed(rotateSpeed);
                    mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
    //                telemetry.addData("rotations (mm?)=", mm);
    //                telemetry.update();
                }
            }
            else if (degrees < 0) {
                while (mm > degrees) {
                    //elevatorheight();
                    board.setRotateSpeed(-rotateSpeed);
                    mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
     //               telemetry.addData("rotations (mm?)=", mm);
    //                telemetry.update();
                }
            }
            board.setRotateSpeed(0);
        }
     */
    public void ducksSleepMilliSec(int sleepTime){
        try {
            Thread.sleep(sleepTime);
        } catch(InterruptedException e){
            throw new RuntimeException(e);
        }
    }

    public void MoveForwardDistance(double distance, double forwardSpeed){
//        telemetry.addData("rotations forward", board.getMotorRotations());
        telemetry.update();
        double initialWheelRotation = board.getMotorRotations();
        double millimeters = (forwardconstant * (board.getMotorRotations()-initialWheelRotation));
//        telemetry.addData("millimeters", millimeters);
//        telemetry.update();
        if (distance > 0) {
            while (millimeters < distance) {
                //elevatorheight();
                if (millimeters > distance* 3/4){
                    forwardSpeed = .2;
                }

                board.setForwardSpeed(forwardSpeed);
                millimeters = (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", millimeters);
                telemetry.update();
            }
        }
        else if (distance < 0) {
            while (millimeters > distance) {
                //elevatorheight();
                board.setForwardSpeed(-forwardSpeed);
                millimeters = (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
//                telemetry.addData("millimeter slow", millimeters);
//                telemetry.update();
            }
        }
        board.setForwardSpeed(0);
    }





    public void MoveSidewaysDistance(double distance) {
        double initialWheelRotation = board.getMotorRotations();
        double xx = Math.abs(sideconstant * (board.getMotorRotations() - initialWheelRotation));
        if (distance > 0) {
            while (xx < distance) {
                board.setSideMotorSpeed(.2);
                xx = (sideconstant * (board.getMotorRotations() - initialWheelRotation));
//                telemetry.addData("millimeter slow", xx);
//                telemetry.update();
            }
        }
        else if (distance < 0) {
            while (xx > distance) {
                board.setSideMotorSpeed(-.2);
                xx = (sideconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", xx);
                telemetry.update();
            }
        }
        board.setForwardSpeed(0);
    }
    public void MoveArmDegrees(double degrees, double Speed){
        double initialArmRotation = board.getArmMotorRotations();
        double angleDeg = Math.abs(armconstant * (board.getArmMotorRotations() - initialArmRotation));
        if (degrees > 0) {
            while (angleDeg < degrees) {
                board.setArmSpeed(Speed);
                angleDeg = (armconstant * (board.getArmMotorRotations() - initialArmRotation));
                telemetry.addData("arm angle (degrees) ", angleDeg);
                telemetry.update();
            }
        }
        else if (degrees < 0) {
            while (angleDeg > degrees) {
                board.setArmSpeed(-Speed);
                angleDeg = (armconstant * (board.getArmMotorRotations() - initialArmRotation));
                telemetry.addData("arm angle (degrees) ", angleDeg);
                telemetry.update();
            }
        }
        board.setArmSpeed(0);
        telemetry.addData("degrees?", angleDeg);
        telemetry.update();
    }

    public void leftPlacement () {
        visionPortal.close();
        MoveSidewaysDistance(-95);
        ducksSleepMilliSec(1000);
        MoveForwardDistance(950, 0.4);
        ducksSleepMilliSec(1000);
        MoveForwardDistance(-205, 0.2);
        board.setClawRotation(0.0);
        MoveArmDegrees(-13, 0.3);
        ducksSleepMilliSec(1000);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        board.setClaw_1Inactive();
        ducksSleepMilliSec(1000);
        MoveArmDegrees(15, 0.3);
        board.setClawRotation(0.7);
        MoveForwardDistance(-75, 0.2);
        ducksSleepMilliSec(500);
        MoveRotateDegrees(-90, 0.1);
        ducksSleepMilliSec(500);
    }
    public void centerPlacement () {
        visionPortal.close();
        MoveForwardDistance(1050, 0.4);
        ducksSleepMilliSec(500);
        MoveForwardDistance(-175, 0.2);
        ducksSleepMilliSec(500);
        board.setClawRotation(0.0);
        MoveArmDegrees(-13, 0.3);
        ducksSleepMilliSec(500);
        board.setClaw_1Inactive();
        ducksSleepMilliSec(500);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        MoveArmDegrees(15, 0.3);
        board.setClawRotation(0.7);
        MoveForwardDistance(-50, 0.2);
        ducksSleepMilliSec(500);
        MoveRotateDegrees(-91, 0.1);
        ducksSleepMilliSec(1000);

    }
    public void rightPlacement() {
        //MoveSidewaysDistance(-90);
        visionPortal.close();
        MoveForwardDistance(800, 0.5);
        ducksSleepMilliSec(1000);
        MoveRotateDegrees(90, 0.1);
        ducksSleepMilliSec(600);
        MoveForwardDistance(380, 0.4);
        ducksSleepMilliSec(500);
        MoveForwardDistance(-195, 0.4);
        ducksSleepMilliSec(500);
        board.setClawRotation(0.0);
        MoveArmDegrees(-13, 0.5);
        ducksSleepMilliSec(500);
        board.setClaw_1Inactive();
        ducksSleepMilliSec(100);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        MoveArmDegrees(15, 0.5);
        ducksSleepMilliSec(500);
        MoveForwardDistance(-50, 0.4);
        ducksSleepMilliSec(500);
        MoveRotateDegrees(180, 0.1);
        ducksSleepMilliSec(500);
        MoveForwardDistance(100, 0.2);
        ducksSleepMilliSec(500);
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                if (detection.id == 1){
                    id_1_x_position = detection.ftcPose.x;
                    id_1_y_position = detection.ftcPose.y;
                }
                if (detection.id == 2){
                    id_2_x_position = detection.ftcPose.x;
                    id_2_y_position = detection.ftcPose.y;
                }
                if (detection.id == 3){
                    id_3_x_position = detection.ftcPose.x;
                    id_3_y_position = detection.ftcPose.y;
                }
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }


        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

}

/*
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            VisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        }
        else {
            VisionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }


 */