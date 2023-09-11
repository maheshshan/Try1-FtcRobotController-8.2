package org.firstinspires.ftc.teamcode.VelocityPID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*

 * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup

 */
@TeleOp
public class MotorVelocityPID extends LinearOpMode {
    double Kp = 0.1;
    double Ki = 0.0;
    double Kd = 0.0;
    double integralSum = 0;
    private double lastError = 0;
    private double lastReference =0;

    double a = 0.8; // a can be anything from 0 < a < 1
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;

    ElapsedTime timer = new ElapsedTime();
    DcMotorEx motor;

    double maxVelocity =0;
    double currentVelocity=0;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "rightFront");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double power;

        waitForStart();

        while (opModeIsActive()) {
            power = PIDControl(800, motor.getVelocity());
            //motor.setPower(power);

            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", power);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }

    public double  PIDControl(double reference, double state){
        double error = reference-state;

        double maxIntegralSum =reference;

        // sum of all error over time
        integralSum += error/10*timer.seconds();

        // max out integral sum
        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }

        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        // reset integral sum upon setpoint changes
        if (reference != lastReference) {
            integralSum = 0;
        }

        double errorChange = (error - lastError);
        // filter out hight frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        double derivative = currentFilterEstimate / timer.seconds();



        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);


        lastError = error;
        lastReference = reference;

        // reset the timer for next time
        timer.reset();
        return out;
    }

 }
