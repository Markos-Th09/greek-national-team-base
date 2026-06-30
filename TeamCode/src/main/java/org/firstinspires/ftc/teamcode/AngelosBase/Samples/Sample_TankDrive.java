package org.firstinspires.ftc.teamcode.AngelosBase.Samples;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AngelosBase.Subsystems.TankDrive;
import org.firstinspires.ftc.teamcode.AngelosBase.Util.GamepadEx;

@Disabled // Remove this to see it on the Driver Station
@TeleOp(name="Sample_TankDrive", group="Samples")
public class Sample_TankDrive extends LinearOpMode {
    private GamepadEx controller;
    private TankDrive drivetrain;

    @Override
    public void runOpMode() {
        // 1. Initialize Gamepad Wrapper
        controller = new GamepadEx(gamepad1);

        // 2. Initialize Drivetrain using the "Functional Philosophy"
        // We map the sticks and triggers directly here.
        drivetrain = new TankDrive(
                hardwareMap,
                new MultipleTelemetry(telemetry),
                controller::getLeftStickY,  // Forward/Backward (Inverted for FTC)
                controller::getRightStickX,  // Turning
                controller::getRightTrigger  // Acceleration/Turbo mapping
        );

        telemetry.addLine("Drivetrain Sample Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // 3. The Core Loop
            controller.update(); // Update joystick/button values

            drivetrain.update(); // Calculations: Feedforward + Power Mapping

            telemetry.update();  // Send motor powers and mapping info to driver hub
        }
    }
}