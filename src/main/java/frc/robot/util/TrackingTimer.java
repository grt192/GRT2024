package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * A wrapper around a `Timer` that tracks whether it is started.
 */
public class TrackingTimer extends Timer {
    private boolean started = false;

    @Override
    public void start() {
        started = true;
        super.start();
    }

    @Override
    public void stop() {
        started = false;
        super.stop();
    }

    /**
     * Gets whether the timer has started.
     * @return Whether the timer has started.
     */
    public boolean hasStarted() {
        return started;
    }
}
