package org.firstinspires.ftc.teamcode.AngelosBase.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class ActiveIntake {
    // ----------------------------------- User Configuration ----------------------------------- //
    private static final boolean TELEMETRY_ENABLED = true;
    private static final String INTAKE_MOTOR_NAME = "[INTAKE_MOTOR_NAME]";
    private static final double FORWARD_VELOCITY = 1.0, REVERSE_VELOCITY = -1.0;
    private static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD; // Set to REVERSE if intake is reversed
    private static final DcMotorEx.ZeroPowerBehavior INTAKE_ZERO_POWER_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.BRAKE; // Set to FLOAT if you want the intake to coast when stopped



    // ---------------------------------------- Hardware ---------------------------------------- //
    private DcMotorEx intakeMotor;
    private BooleanSupplier forwardButton, stopButton, reverseButton;

    // ------------------------------------ State Management ------------------------------------ //
    public enum IntakeState {
        STOPPED(0),
        FORWARD(1),
        REVERSE(2);

        private final int idx;
        public static final double[] VELOCITIES = {0.0, FORWARD_VELOCITY, REVERSE_VELOCITY};

        IntakeState(int idx) {
            this.idx = idx;
        }

        public double getVelocity() {
            return VELOCITIES[idx];
        }
    }

    private IntakeState state = IntakeState.STOPPED;

    private boolean disabled;

    private Telemetry telemetry;

    // ------------------------------------------------------------------------------------------ //

    public ActiveIntake(HardwareMap hm,
                        Telemetry telemetry,
                        BooleanSupplier forwardButton,
                        BooleanSupplier stopButton,
                        BooleanSupplier reverseButton
    ) {
        intakeMotor = hm.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        intakeMotor.setZeroPowerBehavior(INTAKE_ZERO_POWER_BEHAVIOR);
        intakeMotor.setDirection(INTAKE_DIRECTION);

        this.forwardButton = forwardButton;
        this.stopButton = stopButton;
        this.reverseButton = reverseButton;

        this.telemetry = telemetry;
    }

    public void update() {
        if (disabled) return;

        // Set Intake State according to button presses
        if(forwardButton.getAsBoolean()) setState(IntakeState.FORWARD);
        if(stopButton.getAsBoolean()) setState(IntakeState.STOPPED);
        if(reverseButton.getAsBoolean()) setState(IntakeState.REVERSE);

        // Set motor power based on state
        setPower(state.getVelocity());

        // -------------------------------------- Telemetry ------------------------------------- //
        if (!TELEMETRY_ENABLED) return;
        telemetry.addData("[Intake] State: ", getState());
    }

    public void setState(IntakeState state) {
        if(this.state == state) return; // No Change, (Less Variable Writing Optimization)
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    // Actuators
    public void intake() {
        setState(IntakeState.FORWARD);
    }

    public void reverse() {
        setState(IntakeState.REVERSE);
    }

    public void stop() {
        setState(IntakeState.STOPPED);
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
