package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Flywheel Tuner")
public class FlywheelTuner extends OpMode {

    // Access values from http://192.168.43.1:8080/dash
    @Config
    public static class Gains {
        public static double kP = 0.0;
        //public static double kI = 0.0;
        //public static double kD = 0.0;
        public static double kF = 0.0;
    }
    public DcMotorEx flywheelMotor;

    public VoltageSensor voltageSensor;
    double targetVelocity = 0;

    //double lastError = 0;
    //double integralSum = 0;

    @Override
    public void init() {
        // Initialize flywheel motor
        // WARNING! set the device name (currently set to "flywheel") to what
        // is set in your driver hub.--------------------------------VVVVVVVV
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        // Set flywheel run mode to not use encoder (takes power as input instead of velocity)
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reverses flywheel direction
        // flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        // Get gamepad commands
        // Turn off flywheel
        if (gamepad1.xWasPressed()) targetVelocity = 0;
        // Turn on flywheel
        if (gamepad1.yWasPressed()) targetVelocity = 1160;

        // Read velocity, voltage, and get error
        double batteryVoltage = voltageSensor.getVoltage();
        double currentVelocity = flywheelMotor.getVelocity();
        double powerFraction = getPowerFraction(currentVelocity, batteryVoltage);
        // Makes sure value doesn't go over or below -1.0 & 1.0
        

        // Sets motor power
        flywheelMotor.setPower(powerFraction);

        // Update telemetry
        // Voltage warning if voltage is < 11.5V
        if (batteryVoltage < 11.5) telemetry.addLine("WARNING! BATTERY VOLTAGE LOW!");
        
        telemetry.addData("Current velocity %.1f", currentVelocity);
        telemetry.addData("Target velocity", targetVelocity);
        telemetry.addData("Error", targetVelocity - currentVelocity);
        telemetry.addLine();
        telemetry.addData("P = %.5f", Gains.kP);
        //telemetry.addData("I = %.5f", Gains.kI);
        //telemetry.addData("D = %.5f", Gains.kD);
        telemetry.addData("F = %.5f", Gains.kF);
        telemetry.update();
    }

    private double getPowerFraction(double currentVelocity, double batteryVoltage) {
        double error = targetVelocity - currentVelocity;

        // PIDF Control Loop
        // Proportional (kP) reacts to current error
        double desiredVoltage = getDesiredVoltage(error);
        // Power fraction to pass to the motor
        if (targetVelocity == 0) return 0;
        return Math.max(-1.0, Math.min(1.0, desiredVoltage / batteryVoltage));

    }

    private double getDesiredVoltage(double error) {
        double proportionalVoltage = Gains.kP * error;
        // Integral (kI) corrects accumulated error
        //integralSum += error;
        //double integralVoltage = Gains.kI * integralSum;
        // Derivative (kD)
        //double derivativeVoltage = Gains.kD * (error - lastError);
        //lastError = error;
        // Feedforward voltage (kF) predicts voltage for target speed
        double feedforwardVoltage = Gains.kF * targetVelocity;

        // Desired voltage
        return feedforwardVoltage + proportionalVoltage;
    }
}
