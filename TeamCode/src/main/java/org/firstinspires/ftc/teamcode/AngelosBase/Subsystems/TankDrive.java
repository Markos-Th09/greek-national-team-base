package org.firstinspires.ftc.teamcode.AngelosBase.Subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AngelosBase.Config.DriveBaseConfig;
import org.firstinspires.ftc.teamcode.AngelosBase.Config.HardwareMapConfig;

import java.util.function.DoubleSupplier;

public class TankDrive {
    // ----------------------------------- User Configuration ----------------------------------- //
    private final boolean TELEMETRY_ENABLED = true;
    private static final DcMotorEx.Direction LEFT_MOTOR_DIRECTION = DcMotorEx.Direction.FORWARD; // Set to REVERSE if left motor is reversed
    private static final DcMotorEx.Direction RIGHT_MOTOR_DIRECTION = DcMotorEx.Direction.FORWARD; // Set to REVERSE if right motor is reversed
    private static final DcMotorEx.ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.BRAKE; // Set to FLOAT if you want the robot to coast when no power is applied



    // ---------------------------------------- Hardware ---------------------------------------- //
    private DcMotorEx leftMotor, rightMotor;
    private DoubleSupplier forwardPower, turnPower, accelPower;
    private boolean disabled;
    private MultipleTelemetry telemetry;

    // ------------------------------------------------------------------------------------------ //
    private double max_mapping_power, left_power, right_power, max_raw_power,
            leftPower_map, rightPower_map;

    public enum Motor {
        LEFT(0),
        RIGHT(1);

        private final int idx;

        Motor(int idx) {
            this.idx = idx;
        }

        public double getKS() {
            double[] KS = {DriveBaseConfig.LEFT_KS, DriveBaseConfig.RIGHT_KS};
            return KS[idx];
        }
        public double getKV() {
            double[] KV = {DriveBaseConfig.LEFT_KV, DriveBaseConfig.RIGHT_KV};
            return KV[idx];
        }
    }

    // ------------------------------------------------------------------------------------------ //

    public TankDrive(
            HardwareMap hm,
            MultipleTelemetry telemetry,
            DoubleSupplier forwardPower,
            DoubleSupplier turnPower,
            DoubleSupplier accelPower
    ) {
        leftMotor = hm.get(DcMotorEx.class, HardwareMapConfig.LEFT_MOTOR_NAME);
        rightMotor = hm.get(DcMotorEx.class, HardwareMapConfig.RIGHT_MOTOR_NAME);

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
                DriveBaseConfig.DEFAULT_POWER,
                DriveBaseConfig.MAX_POWER
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
        ) + DriveBaseConfig.KS_THETA * Math.signum(turnPower.getAsDouble());
        rightPower_map = Range.scale(
                right_power/max_raw_power,
                0.0,
                1.0,
                0.0,
                max_mapping_power
        ) - DriveBaseConfig.KS_THETA * Math.signum(turnPower.getAsDouble());

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

    public void tuneFeedForward() {
        leftMotor.setPower(feedforward(DriveBaseConfig.TUNING_POWER, Motor.LEFT));
        rightMotor.setPower(feedforward(DriveBaseConfig.TUNING_POWER, Motor.RIGHT));

        telemetry.addData("Left Actual Vel: ", leftMotor.getVelocity());
        telemetry.addData("Right Actual Vel: ", rightMotor.getVelocity());
        telemetry.update();
    }
}
