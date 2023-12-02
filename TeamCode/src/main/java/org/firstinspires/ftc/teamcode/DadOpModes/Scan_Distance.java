/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.DadOpModes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
@TeleOp(name = "Sensor: ScanDistance", group = "Sensor")
//@Disabled
public class Scan_Distance extends LinearOpMode {

    private DistanceSensor sensorDistance;
    Servo servo;
    static final int points = 15;
    static final double MIN_POS=0.0;
    static final double MAX_POS = 1.0*140/270; //1.0;

    static final double INCREMENT =  (MAX_POS-MIN_POS)/(points -1);   //0.05;  //.01
    double position = 0; //(MAX_POS-MIN_POS)/2;
    int index = 0;
    static final int CYCLE_MS = 50;
    //1000;  //50
    boolean rampUp =true;
    double[] distance_array = new double[points];
    double[] servo_array = new double[points];

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");  // "servo_distance"
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        servo = hardwareMap.get(Servo.class,"servo_distance");

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            if (rampUp) {    // Keep stepping up until we hit the max value.
                index += 1;
                position = MIN_POS + index*INCREMENT;    //+= INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {    // Keep stepping down until we hit the min value.
                index -= 1;
                position = MIN_POS + index*INCREMENT; //-= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            distance_array[index] = sensorDistance.getDistance(DistanceUnit.CM);
            servo_array[index] = position;
            for(int i=0; i<points; i++){
                telemetry.addData(String.format("servo=%.2f", servo_array[i]),
                        String.format(" range=%.1f cm", distance_array[i]) );
            }
            telemetry.addData("Servo Position", "%5.2f", position);
//            telemetry.addData("deviceName", sensorDistance.getDeviceName() );
//            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
//            telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
//            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
//            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();

            servo.setPosition(position);
            sleep(CYCLE_MS);
            idle();

        }
    }

}
