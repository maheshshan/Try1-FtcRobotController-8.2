package org.firstinspires.ftc.teamcode.VelocityPID;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;


    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "rightFront");
        waitForStart();


        while (opModeIsActive()) {

            //motor.setPower(1);
            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();

            motor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
            motor.setPositionPIDFCoefficients(5.0);
            motor.setVelocity(1000);
        }
    }
}
