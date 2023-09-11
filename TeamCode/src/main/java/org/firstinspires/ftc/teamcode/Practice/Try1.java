package org.firstinspires.ftc.teamcode.Practice;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Try1 extends LinearOpMode {
    DcMotorEx motor;
    double maxVelocity=0;
    @Override
    public void runOpMode() {

        motor= hardwareMap.get(DcMotorEx.class, "rightFront");

        waitForStart();

        while(opModeIsActive())
        {
             motor.setPower(1);

             if(maxVelocity< motor.getVelocity()) {
                 maxVelocity=motor.getVelocity();
             }

             telemetry.addData("Current Velocity",motor.getVelocity());
             telemetry.addData("Max Velocity", maxVelocity);
             telemetry.update();
        }
    }
}
