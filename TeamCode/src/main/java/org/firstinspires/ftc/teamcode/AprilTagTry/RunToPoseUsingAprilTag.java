/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.AprilTagTry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp (name = "Concept: Run To Pose Using AprilTag", group = "Concept")

@Config
public class RunToPoseUsingAprilTag extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Drive2Tag drive2Tag;

    // Use to switch between Manual and Auto. Manual mode helps to drive robot to new position and activating auto to run to given position.
    enum Mode {
        Auto,
        Manual
    }

    private Mode mode;

    // Values for robots target position, robot will run to this position
    public static double X_Position = 0;
    public static double Y_Position = 12;
    public static double Yaw_Angle = 0;  // Yaw Angle can not be bigger or smaller where camera can not see the AprilTag.

        // Values for motor power or drive
        private double X_Drive =0;
        private double Y_Drive =0;
        private double Yaw_Turn =0;

    // Current pose estimates from April tag
    private double X_Current =0;
    private double Y_Current =0;
    private double Yaw_Current =0;

// last error or last distance for PID control.
        private double lastMagnitude = 0;

    // PID parameters to regulate the drive
    public static double Kp = 0.055;
    public static double Kd = 0.01;
    public static double turnKp = 0.01;

    //derivative filter to reduce noise
    private double a = 0.8; // a can be anything from 0 < a < 1
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;

    // Elapsed timer class from SDK for derivative
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    // Vision parameters
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        
        // Codes added for Drive
        mode = Mode.Manual; // switch will be used to swap manual to auto and back
        drive = new SampleMecanumDrive(hardwareMap); // class used from road runner, which simplifies our code required for drive
        drive2Tag = new Drive2Tag(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("mode", mode);

                switch (mode) {
// case Manual starts here
                    case Manual:
            //Power given to motors using SampleMecanumDrive class
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );
                        telemetryAprilTag();

                        if (gamepad1.b) {
                            mode = Mode.Auto;
                        }
                        break;
// case Auto starts here
                    case Auto:

                        // Protection for random number input
                        if (Y_Position<5.0||Y_Position>35)Y_Position=5.0;
                        if (X_Position<-5.0||X_Position>5)X_Position=0.0;
                        if(Yaw_Angle >5 || Yaw_Angle <-5)Yaw_Angle=0;

                        //Function to get current position using AprilTag.
                        telemetryAprilTag();

            //Function to calculate required power
                        calPower();

                //Power given to motors using Drive2Tag class, which set the velocity to motor; power does not help to drive to target.
                            drive2Tag.DrivePower(
                                    Y_Drive,// change direction based on robots run
                                    X_Drive,// change direction based on robots run
                                    Yaw_Turn // change direction based on robots run
                            );

                        telemetry.addLine(String.format("Timer seconds:%6.2f",timer2.seconds()));
                        telemetry.addLine(String.format("Drive Y:%6.2f, X:%6.2f , Yaw: %6.2f", Y_Drive,X_Drive,Yaw_Turn ));
                        if (gamepad1.y) {
                            mode = Mode.Manual;
                        }
                        break;
                }
                telemetry.update();
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
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

    /**
     * Function to add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(currentDetections.size()<1){
            X_Current=X_Position;
            Y_Current=X_Position;
            Yaw_Current=Yaw_Angle;
        }

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                X_Current=detection.ftcPose.x;
                Y_Current=detection.ftcPose.y;
                Yaw_Current=detection.ftcPose.yaw;

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", X_Current, Y_Current, Yaw_Current));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXY Yaw = X (Right), Y (Forward), Yaw.");

    }   // end method telemetryAprilTag()

    public void calPower()
    {
        double magnitude = 0;
        double multiplier = 0;

        magnitude=Math.hypot((X_Current-X_Position), (Y_Current-Y_Position));
        //if(initialMagnitude==0)initialMagnitude=magnitude;

        multiplier=PIDControl (magnitude);

        if (magnitude!=0) {
            X_Drive = (X_Current-X_Position)/magnitude;
            Y_Drive = (Y_Current - Y_Position)/magnitude;
        }else
        {
            X_Drive =0;
            Y_Drive =0;
        }

        X_Drive = X_Drive*multiplier;
        Y_Drive = Y_Drive*multiplier;

        if(Yaw_Current!=Yaw_Angle) {
            //if(initialYawAngle==0)initialYawAngle=Math.abs(Yaw_Current - Yaw_Angle);

            Yaw_Turn = (Yaw_Angle-Yaw_Current); // Math.abs(initialYawAngle);
            Yaw_Turn= Yaw_Turn*turnKp;
        }
        else Yaw_Turn =0;
    }

    public double PIDControl (double error)
    {
        double multiplier = 0;
        double derivative =0;
        //double integralSum = 0; // integral portion does not help since no requirement to hold target.
        double errorChange =0;

        // sum of all error over time
        //integralSum = integralSum + (error * timer.seconds());

        errorChange = (error - lastMagnitude);

        // filter out high frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        derivative = currentFilterEstimate / timer.seconds();

        multiplier= (Kp * error) + (Kd * derivative); //+ (Ki * integralSum)
        lastMagnitude = error;

        // reset the timer for next time
        timer.reset();

        return multiplier;
    }

}   // end class
