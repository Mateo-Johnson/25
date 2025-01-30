package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Copperforge Lasershark LiDAR ranging sensor.
 * filtering, validity checking, and movement detection capabilities.
 */
public class Lasershark implements Sendable {
    private DutyCycle _pwmInput;
    private MedianFilter filter;
    private double lastDistance = 0.0;
    private double lastValidReading = 0.0;
    private double lastReadingTime = 0.0;
    private boolean filterEnabled = false;
    private static final double min_valid = 2.0; // Min spec is 2 inches
    private static final double max_valid = 144.0; // Max spec is 12 feet
    
    // Movement detection thresholds
    private static final double MOVEMENT_THRESHOLD_INCHES = 1.0;
    private static final double RAPID_MOVEMENT_THRESHOLD_INCHES = 6.0;

    /**
     * Construct a new Lasershark on a given digital input channel.
     *
     * @param channel The channel to which to attach.
     */
    public Lasershark(int channel) {
        this(new DigitalInput(channel));
    }

    /**
     * Construct a new Lasershark attached to a DigitalSource object.
     *
     * @param source The digital source to which to attach.
     */
    public Lasershark(DigitalSource source) {
        this._pwmInput = new DutyCycle(source);
        this.filter = new MedianFilter(5); // 5-sample median filter
        SendableRegistry.addLW(this, "Lasershark", _pwmInput.getFPGAIndex() + 1);
    }

    /**
     * Enable or disable median filtering of readings.
     *
     * @param enabled True to enable filtering, false to disable
     */
    public void setFilterEnabled(boolean enabled) {
        this.filterEnabled = enabled;
    }

    /**
     * Check if the current reading is valid (within spec range).
     *
     * @return True if the reading is valid, false otherwise
     */
    public boolean isValidReading() {
        double distance = getDistanceInches();
        return distance >= min_valid && distance <= max_valid;
    }

    /**
     * Get the last valid reading if current reading is invalid.
     *
     * @return The last valid reading, or current reading if valid
     */
    public double getDistanceWithFallback() {
        double currentDistance = getDistanceInches();
        if (isValidReading()) {
            lastValidReading = currentDistance;
            return currentDistance;
        }
        return lastValidReading;
    }

    /**
     * Detect if significant movement has occurred since last reading.
     *
     * @return True if movement detected, false otherwise
     */
    public boolean hasMovementOccurred() {
        double currentDistance = getDistanceInches();
        boolean hasMovement = Math.abs(currentDistance - lastDistance) > MOVEMENT_THRESHOLD_INCHES;
        lastDistance = currentDistance;
        return hasMovement;
    }

    /**
     * Detect if rapid movement has occurred since last reading.
     *
     * @return True if rapid movement detected, false otherwise
     */
    public boolean hasRapidMovementOccurred() {
        double currentDistance = getDistanceInches();
        boolean hasRapidMovement = Math.abs(currentDistance - lastDistance) > RAPID_MOVEMENT_THRESHOLD_INCHES;
        lastDistance = currentDistance;
        return hasRapidMovement;
    }

    /**
     * Calculate velocity of target in inches per second.
     *
     * @return Velocity in inches per second, positive means moving away
     */
    public double getVelocity() {
        double currentTime = Timer.getFPGATimestamp();
        double currentDistance = getDistanceInches();
        double deltaTime = currentTime - lastReadingTime;
        double deltaDistance = currentDistance - lastDistance;
        
        // Update stored values
        lastDistance = currentDistance;
        lastReadingTime = currentTime;
        
        if (deltaTime > 0) {
            return deltaDistance / deltaTime;
        }
        return 0.0;
    }

    /**
     * Get the filtered distance reported by the LiDAR sensor in feet.
     *
     * @return The filtered distance reported by the LiDAR sensor in feet.
     */
    public double getDistanceFeet() {
        return getDistanceInches() / 12.0;
    }

    /**
     * Get the filtered distance reported by the LiDAR sensor in inches.
     *
     * @return The filtered distance reported by the LiDAR sensor in inches.
     */
    public double getDistanceInches() {
        double rawOutput = this._pwmInput.getOutput() * 4000 / 25.4;
        if (filterEnabled) {
            return filter.calculate(rawOutput);
        }
        return rawOutput;
    }

    /**
     * Get the filtered distance reported by the LiDAR sensor in centimeters.
     *
     * @return The filtered distance reported by the LiDAR sensor in centimeters.
     */
    public double getDistanceCentimeters() {
        return getDistanceInches() * 2.54;
    }

    /**
     * Get the filtered distance reported by the LiDAR sensor in meters.
     *
     * @return The filtered distance reported by the LiDAR sensor in meters.
     */
    public double getDistanceMeters() {
        return getDistanceCentimeters() / 100.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Lasershark");
        builder.addDoubleProperty("Distance (ft)", this::getDistanceFeet, null);
        builder.addDoubleProperty("Distance (in)", this::getDistanceInches, null);
        builder.addDoubleProperty("Velocity (in/s)", this::getVelocity, null);
        builder.addBooleanProperty("Valid Reading", this::isValidReading, null);
        builder.addBooleanProperty("Movement Detected", this::hasMovementOccurred, null);
    }
}