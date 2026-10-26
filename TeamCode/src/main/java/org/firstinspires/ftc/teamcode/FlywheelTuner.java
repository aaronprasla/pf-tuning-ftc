package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuner extends OpMode {
    public DcMotorEx flywheelMotor;

    public double highVelocity = 1150;
    public double lowVelocity = 1000;
    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        // Initialize flywheel motor
        // WARNING! set the device name (currently set to "flywheel") to what
        // is set in your driver hub.--------------------------------VVVVVVVV
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        // Set flywheel run mode to use encoder (takes velocity as input instead of power)
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Reverses flywheel direction
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Creates PIDFCoefficients object using P & F variables. I & D remain constant
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        // Sets PIDF Coefficients for the motor
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        // Lets user know that initialization is complete
        telemetry.addLine("Init Complete");

    }

    @Override
    public void loop() {
        // Get gamepad commands

        // Switches between low & high velocity if triangle (y) is pressed.
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }
        // Increments step index by one every time circle (b) is pressed.
        // If it goes over, it goes back to the beginning
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Increments F by stepSizes by stepIndex
        if (gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // Set target velocity

        // Set new PIDFCoeff
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set velocity
        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        // Update telemetry
        telemetry.addData("Target Velocity: ", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", " %.2f", error);
        telemetry.addLine("--------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (Circle)", stepSizes[stepIndex]);
        telemetry.addLine("Switch between high & low velocity (Triangle)");
    }
}
