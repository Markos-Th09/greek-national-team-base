package org.firstinspires.ftc.teamcode.AngelosBase.Config;

public class DriveBaseConfig {
    // Feedforward constants
    // KS: is the static gain -> for Static Friction
    // KV: is the velocity gain -> Fixes Motor Inaccuracies Linearly
    public static final double[] LEFT_FEEDFORWARD = {0.03, 1.0}; // KS, KV for left motor
    public static final double[] RIGHT_FEEDFORWARD = {0.03, 1.0}; // KS, KV for right motor
    public static final double KS_THETA = 0.08; // Static gain for turning

    // Power Mapping Constants
    public static final double DEFAULT_POWER = 0.6; // Default power for driving
    public static final double MAX_POWER = 1.0; // Maximum power for driving
}
