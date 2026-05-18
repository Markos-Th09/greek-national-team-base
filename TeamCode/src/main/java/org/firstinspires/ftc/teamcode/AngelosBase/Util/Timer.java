package org.firstinspires.ftc.teamcode.AngelosBase.Util;

public class Timer {
    private long startTime;
    public Timer() {
        resetTimer();
    }
    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }
    public double getCurTimeSecs() {
        return (getCurTime() / 1000.0);
    }
    public long getCurTime() {
        return System.currentTimeMillis() - startTime;
    }
}
