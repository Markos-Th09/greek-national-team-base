package org.firstinspires.ftc.teamcode.AngelosBase.Samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AngelosBase.Util.GamepadEx;
import org.firstinspires.ftc.teamcode.AngelosBase.Util.Timer;

@Disabled
@TeleOp(name="Sample_Timer", group="Samples")
public class Sample_Timer extends LinearOpMode {
    private GamepadEx controller;
    private Timer timer;

    @Override
    public void runOpMode() {
        // 1. Initialize Utilities
        controller = new GamepadEx(gamepad1);
        timer = new Timer();

        telemetry.addLine("Timer Sample Initialized");
        telemetry.addLine("Press A to Reset | Check Status at 5s");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // 2. Refresh Inputs
            controller.update();

            // 3. Timer Reset Logic
            // Using justPressed ensures the timer resets exactly once per click
            if (controller.justPressed(GamepadEx.Button.A)) {
                timer.resetTimer();
            }

            // 4. Time-Based Logic
            double elapsed = timer.getCurTimeSecs();
            boolean isExpired = elapsed > 5.0;

            // 5. Telemetry Feedback
            telemetry.addData("Status", isExpired ? "!!! 5 SECONDS UP !!!" : "Counting...");
            telemetry.addData("Elapsed Time", "%.2f seconds", elapsed);

            if (isExpired) {
                telemetry.addLine("ACTION REQUIRED: Reset timer with A");
            }

            telemetry.update();
        }
    }
}