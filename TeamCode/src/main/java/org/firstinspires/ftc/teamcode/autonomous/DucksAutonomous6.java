package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
public class DucksAutonomous6 extends OpMode {
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

    @Override
    public void init() {
        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class,
                        "Webcam 1"),
                visionProcessor);
//        telemetry.addData("Identified", visionProcessor.getSelection());
//        telemetry.addData("test", visionProcessor.getData());
        telemetry.update();

        board.init(hardwareMap);
        telemetry.addData("rotations init", board.getMotorRotations());
        telemetry.update();
    }

    @Override
    public void init_loop(){
        telemetry.addData("Identified_loop", visionProcessor.getSelection());

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        if (state == 0) {
            board.setClaw_1Active();
            board.setClaw_2Active();
            board.setClawRotation(0);
            state = 1;
        }
        if (state == 1) {
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state = 2;
        }
        if (state == 2){
            MoveArmDegrees(15,0.3);
            state = 3;
        }
        if (state == 3){
            board.setClawRotation(0.7);
            state = 4;
        }
        if (state == 4){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state = 5;
        }
        if (state == 5){
            MoveSidewaysDistance(140);
            state = 6;
        }
        if (state == 6){
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state = 7;
        }
        if (state == 7){
            Position = visionProcessor.getSelection();

            if (Position .equals(FirstVisionProcessor.Selected.MIDDLE)){
                position = 2;
                centerPlacement();
                state = 8;
            }
            if (Position == FirstVisionProcessor.Selected.LEFT) {
                position = 1;
                MoveSidewaysDistance(-140);
                leftPlacement();
                state = 8;
            }
            else {
                position = 3;
                MoveSidewaysDistance(-140);
                rightPlacement();
                state = 8;
            }
        }
        if (state == 8){
            visionPortal.stopStreaming();
            state = 9;
        }

    }

    public void MoveForwardDistance(double distance, double forwardSpeed){
        telemetry.addData("rotations forward", board.getMotorRotations());
        telemetry.update();
        double initialWheelRotation = board.getMotorRotations();
        double millimeters = (forwardconstant * (board.getMotorRotations()-initialWheelRotation));
        telemetry.addData("millimeters", millimeters);
        telemetry.update();
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
                telemetry.addData("millimeter slow", millimeters);
                telemetry.update();
            }
        }
        board.setForwardSpeed(0);
    }
    public void MoveRotateDegrees(double degrees, double rotateSpeed) {
        double initialWheelRotation = board.getMotorRotations();
        double mm = (rotationConstant * (board.getMotorRotations()-initialWheelRotation));
        if (degrees > 0) {
            while (mm < degrees) {
                //elevatorheight();
                board.setRotateSpeed(rotateSpeed);
                mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("rotations (mm?)=", mm);
                telemetry.update();
            }
        }
        else if (degrees < 0) {
            while (mm > degrees) {
                //elevatorheight();
                board.setRotateSpeed(-rotateSpeed);
                mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("rotations (mm?)=", mm);
                telemetry.update();
            }
        }
        board.setRotateSpeed(0);
    }
    public void MoveSidewaysDistance(double distance) {
        double initialWheelRotation = board.getMotorRotations();
        double xx = Math.abs(sideconstant * (board.getMotorRotations() - initialWheelRotation));
        if (distance > 0) {
            while (xx < distance) {
                board.setSideMotorSpeed(.2);
                xx = (sideconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", xx);
                telemetry.update();
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
        int state_left = 0;

        if (state_left == 0) {
            MoveSidewaysDistance(-90);
            state_left =1;
        }
        if (state_left == 1) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_left = 2;
        }
        if (state_left == 2) {
            MoveForwardDistance(400, 0.4);
            state_left = 3;
        }
        if (state_left == 3) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_left = 4;
        }
        if (state_left == 4){
            MoveArmDegrees(-13, 0.3);
            state_left = 5;
        }
        if (state_left == 5){
            board.setClawRotation(0.0);
            state_left = 6;
        }
        if (state_left == 6){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_left = 7;
        }
        if (state_left == 7) {
            board.setClaw_1Inactive();
            state_left = 8;
        }
        if (state_left == 8) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_left = 9;
        }
        if (state_left == 9){
            MoveArmDegrees(15, 0.3);
            state_left = 10;
        }
        if (state_left == 10) {
            MoveForwardDistance(-200, 0.4);
            state_left = 11;
        }
        if (state_left == 11){
            MoveRotateDegrees(-90, 0.1);
            state_left = 12;
        }

    }
    public void centerPlacement () {
        int state_center = 0;

        if (state_center == 0) {
            MoveForwardDistance(1050, 0.4);
            state_center = 1;
        }
        if (state_center == 1){
            MoveForwardDistance(-200, 0.4);
            state_center = 2;
        }
        if (state_center == 2) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_center = 3;
        }
        if (state_center == 3) {
            board.setClawRotation(0.0);
            state_center = 4;
        }
        if (state_center == 4){
            MoveArmDegrees(-13, 0.3);
            state_center = 5;
        }
        if (state_center == 5) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_center = 6;
        }
        if (state_center == 6){
            board.setClaw_1Inactive();
            state_center = 7;
        }
        if (state_center == 7) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_center = 8;
        }
        if (state_center == 8){
            MoveArmDegrees(15, 0.3);
            state_center = 9;
        }
        if (state_center == 9){
            MoveForwardDistance(-100, 0.4);
            state_center = 10;
        }
        if (state_center == 10){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_center = 11;
        }
        if (state_center == 11){
            MoveRotateDegrees(-90, 0.1);
            state_center = 12;
        }
        if (state_center == 12){
            board.setSideMotorSpeed(0.0);
            board.setForwardSpeed(0);
            state_center = 13;
        }

    }
    public void rightPlacement() {
        int state_right = 0;
        if (state_right == 0){
            MoveSidewaysDistance(-90);
            state_right = 1;
        }
        if (state_right == 1){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_right = 2;
        }
        if (state_right == 2){
            MoveForwardDistance(800, 0.4);
            state_right = 3;
        }
        if (state_right == 3){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_right = 4;
        }
        if (state_right == 4){
            MoveRotateDegrees(90, 0.1);
            state_right = 5;
        }
        if (state_right == 5){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_right = 6;
        }
        if (state_right == 6){
            MoveForwardDistance(550, 0.4);
            state_right = 7;
        }
        if (state_right == 7){
            MoveForwardDistance(-200, 0.3);
            state_right = 8;
        }
        if (state_right == 8){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_right = 9;
        }
        if (state_right == 9){
            board.setClawRotation(0.0);
            state_right = 10;
        }
        if (state_right == 10){
            MoveArmDegrees(-13, 0.3);
            state_right = 11;
        }
        if (state_right == 11){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_right = 12;
        }
        if (state_right == 12){
            board.setClaw_1Inactive();
            state_right = 13;
        }
        if (state_right == 13){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_right = 14;
        }
        if (state_right == 14){
            MoveForwardDistance(-300, 0.4);
            state_right = 15;
        }
        if (state_right == 15){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state_right = 16;
        }
        if (state_right == 16){
            MoveRotateDegrees(180, 0.1);
            state_right = 17;
        }
    }


}

