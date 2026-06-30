package org.firstinspires.ftc.teamcode.AngelosBase.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveBaseConfig {
    // Feedforward constants
    // KS: is the static gain -> for Static Friction
    // KV: is the velocity gain -> Fixes Motor Inaccuracies Linearly
    public static double LEFT_KS = 0.03;
    public static double LEFT_KV = 1.0;

    public static double RIGHT_KS = 0.03;
    public static double RIGHT_KV = 1.0;
    public static double KS_THETA = 0.08; // Static gain for turning

    // Power Mapping Constants
    public static double DEFAULT_POWER = 0.6; // Default power for driving
    public static double MAX_POWER = 1.0; // Maximum power for driving


    public static double POWER = 0.0;
    public static final int TICKS_PER_REVOLUTION = 28;
    public static final int MAX_RPM = 500;

    public static double UTURN_THRESHOLD_DEG = 3.0;
}
