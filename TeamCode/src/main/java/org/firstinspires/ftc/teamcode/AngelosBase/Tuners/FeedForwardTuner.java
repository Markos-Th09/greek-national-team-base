package org.firstinspires.ftc.teamcode.AngelosBase.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AngelosBase.Config.HardwareMapConfig;
import org.firstinspires.ftc.teamcode.AngelosBase.Config.DriveBaseConfig;

// This TEST FILE: tunes the feedforward values for the drive base motors
// (++ FTCDashboard Config Variables)

//@Disabled // TEST FILE: It is Disabled the OpMode so it doesn't show up in the driver station
@TeleOp(name = "FeedForwardTuner", group = "Tuners")
public class FeedForwardTuner extends LinearOpMode {
    private DcMotorEx leftMotor, rightMotor;
    private static final double EPSILON = 1e-23;
    private Telemetry dashTele;

    private double feedforward(double power, double ks, double kv) {
        return ks * Math.signum(power + EPSILON) + kv * power;
    }

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapConfig.LEFT_MOTOR_NAME);
        rightMotor = hardwareMap.get(DcMotorEx.class, HardwareMapConfig.RIGHT_MOTOR_NAME);

        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        dashTele = FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            leftMotor.setPower(feedforward(DriveBaseConfig.POWER, DriveBaseConfig.LEFT_KS, DriveBaseConfig.LEFT_KV));
            rightMotor.setPower(feedforward(DriveBaseConfig.POWER, DriveBaseConfig.RIGHT_KS, DriveBaseConfig.RIGHT_KV));
            dashTele.addData("Target Vel: ",
                    DriveBaseConfig.POWER * DriveBaseConfig.TICKS_PER_REVOLUTION * DriveBaseConfig.MAX_RPM
            );
            dashTele.addData("Left Actual Vel: ", leftMotor.getVelocity());
            dashTele.addData("Right Actual Vel: ", rightMotor.getVelocity());
            dashTele.update();
        }
    }
}
