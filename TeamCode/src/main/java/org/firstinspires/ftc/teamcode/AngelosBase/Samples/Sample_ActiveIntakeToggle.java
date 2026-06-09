package org.firstinspires.ftc.teamcode.AngelosBase.Samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AngelosBase.Subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.AngelosBase.Util.GamepadEx;

@Disabled // Remove this to see it on the Driver Station
@TeleOp(name="Sample_ActiveIntakeToggle", group="Samples")
public class Sample_ActiveIntakeToggle extends LinearOpMode {
    private GamepadEx controller;
    private ActiveIntake intake;

    @Override
    public void runOpMode() {
        // Initialize Gamepad Wrapper
        controller = new GamepadEx(gamepad1);

        // Initialize Intake
        // Mapping:
        // A -> Toggle (Forward/Stop)
        // B -> Reverse
        intake = new ActiveIntake(
                hardwareMap,
                telemetry,
                () -> controller.justPressed(GamepadEx.Button.A),      // toggle supplier
                () -> controller.justPressed(GamepadEx.Button.B)       // reverse supplier
        );

        telemetry.addLine("Intake Sample Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controller.update(); // Refresh button states

            intake.update();     // Runs the state logic and sets motor power

            telemetry.update();  // Sends [Intake] State to the driver hub
        }
    }
}