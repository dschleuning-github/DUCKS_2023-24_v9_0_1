package org.firstinspires.ftc.teamcode.DadOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import processors.FirstVisionProcessor;

@Autonomous
public class FirstVisionOpMode extends OpMode {
    private FirstVisionProcessor visionProcessor;
//    public double satTest = 0;
//    satTest = getAveSaturation(hsvMat, rectMiddle);

    private VisionPortal visionPortal;
    @Override
    public void init(){
        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class,
                        "Webcam 1"),
                visionProcessor);
//        telemetry.addData("Identified", visionProcessor.getSelection());
//        telemetry.addData("test", visionProcessor.getData());
        telemetry.update();
    }
    @Override
    public void init_loop(){
        double[] xxx = new double[3];
        xxx = visionProcessor.getData();
        telemetry.addData("Identified_loop", visionProcessor.getSelection());
        for(int i=0; i<3; i++) {
            telemetry.addData(String.format("xxx=%.2f", xxx[i]),
                    String.format(" yyy=%.1f cm", xxx[i]));
        }
//        telemetry.addData("test_loop", xxx[0], xxx[1]);
//        telemetry.addData("center", visionProcessor.getCenter());
    }

    @Override
    public void start(){
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Identified", visionProcessor.getSelection());
        telemetry.addData("test", visionProcessor.getData()[0]);
    }


}
