package org.firstinspires.ftc.teamcode.AngelosBase.Util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class GamepadEx {
    private final Gamepad controller;

    public enum Button { // Enum for button mapping
        A, B, X, Y, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON
    }

    private final Map<Button, BooleanSupplier> buttonMap = new EnumMap<>(Button.class);

    private final boolean[] cur_states = new boolean[Button.values().length];
    private boolean[] prev_states = new boolean[Button.values().length];

    public GamepadEx(Gamepad gamepad1) {
        this.controller = gamepad1;

        buttonMap.put(Button.A, () -> controller.a);
        buttonMap.put(Button.B, () -> controller.b);
        buttonMap.put(Button.X, () -> controller.x);
        buttonMap.put(Button.Y, () -> controller.y);
        buttonMap.put(Button.DPAD_UP, () -> controller.dpad_up);
        buttonMap.put(Button.DPAD_DOWN, () -> controller.dpad_down);
        buttonMap.put(Button.DPAD_LEFT, () -> controller.dpad_left);
        buttonMap.put(Button.DPAD_RIGHT, () -> controller.dpad_right);
        buttonMap.put(Button.LEFT_BUMPER, () -> controller.left_bumper);
        buttonMap.put(Button.RIGHT_BUMPER, () -> controller.right_bumper);
        buttonMap.put(Button.LEFT_STICK_BUTTON, () -> controller.left_stick_button);
        buttonMap.put(Button.RIGHT_STICK_BUTTON, () -> controller.right_stick_button);
    }

    public void update() {
        prev_states = cur_states.clone();
        for (Button button : Button.values()) {
            cur_states[button.ordinal()] = buttonMap.get(button).getAsBoolean();
        }
    }

    // ---------------------------------- Rising Edge Detection --------------------------------- //
    public boolean justPressed(@NonNull Button button) {
        return cur_states[button.ordinal()] && !prev_states[button.ordinal()];
    }

    // --------------------------------- Falling Edge Detection --------------------------------- //
    public boolean justReleased(@NonNull Button button) {
        return !cur_states[button.ordinal()] && prev_states[button.ordinal()];
    }

    // ------------------------------------- Button States -------------------------------------- //
    public boolean isDown(@NonNull Button button) {
        return cur_states[button.ordinal()];
    }

    public boolean isUp(@NonNull Button button) {
        return !cur_states[button.ordinal()];
    }

    // ----------------------------------------- Linear ----------------------------------------- //
    public double getLeftStickX() {
        return controller.left_stick_x;
    }

    public double getLeftStickY() {
        return -controller.left_stick_y; // Minus to fix the incorrect joystick sign (+ -> up)
    }

    public double getRightStickX() {
        return controller.right_stick_x;
    }

    public double getRightStickY() {
        return -controller.right_stick_y; // Minus to fix the incorrect joystick sign (+ -> up)
    }

    public double getLeftTrigger() {
        return controller.left_trigger;
    }

    public double getRightTrigger() {
        return controller.right_trigger;
    }
}
