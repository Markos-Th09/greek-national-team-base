package org.firstinspires.ftc.teamcode.AngelosBase.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class TankDrive {
    // ----------------------------------- User Configuration ----------------------------------- //
    private final boolean TELEMETRY_ENABLED = true;

    // Motor Configuration
    public static final String LEFT_MOTOR_NAME = "[LEFT_MOTOR_NAME]";
    public static final String RIGHT_MOTOR_NAME = "[RIGHT_MOTOR_NAME]";
    private static final DcMotorEx.Direction LEFT_MOTOR_DIRECTION = DcMotorEx.Direction.FORWARD; // Set to REVERSE if left motor is reversed
    private static final DcMotorEx.Direction RIGHT_MOTOR_DIRECTION = DcMotorEx.Direction.FORWARD; // Set to REVERSE if right motor is reversed
    private static final DcMotorEx.ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.BRAKE; // Set to FLOAT if you want the robot to coast when no power is applied

    // Feedforward constants
    // KS: is the static gain -> for Static Friction
    // KV: is the velocity gain -> Fixes Motor Inaccuracies Linearly
    private static final double[] LEFT_FEEDFORWARD = {0.03, 1.0}; // KS, KV for left motor
    private static final double[] RIGHT_FEEDFORWARD = {0.03, 1.0}; // KS, KV for right motor
    private static final double KS_THETA = 0.08; // Static gain for turning

    // Power Mapping Constants
    private static final double DEFAULT_POWER = 0.6; // Default power for driving
    private static final double MAX_POWER = 1.0; // Maximum power for driving



    // ---------------------------------------- Hardware ---------------------------------------- //
    private DcMotorEx leftMotor, rightMotor;
    private DoubleSupplier forwardPower, turnPower, accelPower;
    private boolean disabled;
    private Telemetry telemetry;

    // ------------------------------------------------------------------------------------------ //
    private double max_mapping_power, left_power, right_power, max_raw_power,
            leftPower_map, rightPower_map;

    public enum Motor {
        LEFT(0),
        RIGHT(1);

        public static final double[] KS = {LEFT_FEEDFORWARD[0], RIGHT_FEEDFORWARD[0]};
        public static final double[] KV = {LEFT_FEEDFORWARD[1], RIGHT_FEEDFORWARD[1]};

        private final int idx;

        Motor(int idx) {
            this.idx = idx;
        }

        public double getKS() {
            return KS[idx];
        }
        public double getKV() {
            return KV[idx];
        }
    }

    // ------------------------------------------------------------------------------------------ //

    public TankDrive(
            HardwareMap hm,
            Telemetry telemetry,
            DoubleSupplier forwardPower,
            DoubleSupplier turnPower,
            DoubleSupplier accelPower
    ) {
        leftMotor = hm.get(DcMotorEx.class, LEFT_MOTOR_NAME);
        rightMotor = hm.get(DcMotorEx.class, RIGHT_MOTOR_NAME);

        leftMotor.setZeroPowerBehavior(MOTOR_ZERO_POWER_BEHAVIOR);
        rightMotor.setZeroPowerBehavior(MOTOR_ZERO_POWER_BEHAVIOR);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        rightMotor.setDirection(RIGHT_MOTOR_DIRECTION);

        this.forwardPower = forwardPower;
        this.turnPower = turnPower;
        this.accelPower = accelPower;

        this.telemetry = telemetry;
    }

    public double feedforward(double input, Motor motor) {
        // Formula: output = Math.signum(input) * Ks + input * kV
        return motor.getKS() * Math.signum(input) + motor.getKV() * input;
    }

    public void update() {
        if (disabled) return;

        // ------------------------------------ Calculations ------------------------------------ //
        max_mapping_power = Range.scale(
                accelPower.getAsDouble(),
                0,
                1,
                DEFAULT_POWER,
                MAX_POWER
        ); // Map the Max Power Based on Trigger

        left_power = forwardPower.getAsDouble() + turnPower.getAsDouble();
        right_power = forwardPower.getAsDouble() - turnPower.getAsDouble();

        max_raw_power = Math.max(
                Math.abs(forwardPower.getAsDouble())+Math.abs(turnPower.getAsDouble()),
                1
        );

        // Per Side/Motor Power Calculation
        leftPower_map = Range.scale(
                left_power/max_raw_power,
                0.0,
                1.0,
                0.0,
                max_mapping_power
        ) + KS_THETA * Math.signum(turnPower.getAsDouble());
        rightPower_map = Range.scale(
                right_power/max_raw_power,
                0.0,
                1.0,
                0.0,
                max_mapping_power
        ) - KS_THETA * Math.signum(turnPower.getAsDouble());

        leftMotor.setPower(feedforward(leftPower_map, Motor.LEFT));
        rightMotor.setPower(feedforward(rightPower_map, Motor.RIGHT));

        // -------------------------------------- Telemetry ------------------------------------- //
        if (!TELEMETRY_ENABLED) return;

        telemetry.addData("[Drivetrain] Forward Power: ", forwardPower.getAsDouble());
        telemetry.addData("[Drivetrain] Turn Power: ", turnPower.getAsDouble());
        telemetry.addData("[Drivetrain] left: ", left_power);
        telemetry.addData("[Drivetrain] right: ", right_power);
        telemetry.addData("[Drivetrain] max raw: ", max_raw_power);
        telemetry.addData("[Drivetrain] left/map: ", left_power/max_raw_power);
        telemetry.addData("[Drivetrain] right/map: ", right_power/max_raw_power);
        telemetry.addData("[Drivetrain] left map: ", leftPower_map);
        telemetry.addData("[Drivetrain] right map: ", rightPower_map);
    }

    // ------------------------------------- Utility Methods ------------------------------------ //
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void enable() {
        setDisabled(false);
    }

    public void disable() {
        setDisabled(true);
    }
}
