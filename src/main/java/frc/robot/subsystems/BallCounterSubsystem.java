package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;


/**
 * BallCounterSubsystem uses a CANrange sensor to count balls passing through.
 */
public class BallCounterSubsystem extends SubsystemBase {

    // CAN ID for the CANrange sensor
    private static final int CANRANGE_ID = 28;

    // Detection threshold in meters (6 inches converted to meters)
    private static final double BALL_DETECTION_THRESHOLD_METERS = Inches.of(6).in(Meters);

    // Target ball count for green indicator
    private static final int TARGET_COUNT = 8;

    private final CANrange canRange;
    private final StatusSignal<Distance> distanceSignal;
    private final StatusSignal<Boolean> isDetectedSignal;
    private final ShuffleboardTab ballCounterTab;

    // Ball counter state
    private int ballCount = 0;
    private boolean ballWasDetected = false; // Track previous state to count only once per ball

    public BallCounterSubsystem() {
        // Initialize CANrange with correct CANBus from TunerConstants
        canRange = new CANrange(CANRANGE_ID, TunerConstants.kCANBus);

        // Configure CANrange
        configureCANrange();

        // Get status signals
        distanceSignal = canRange.getDistance();
        isDetectedSignal = canRange.getIsDetected();

        // Create Shuffleboard tab
        ballCounterTab = Shuffleboard.getTab("Ball Counter");

        // Add widgets to Shuffleboard
        ballCounterTab.addNumber("Ball Count", this::getBallCount)
            .withPosition(0, 0)
            .withSize(3, 2);

        ballCounterTab.addBoolean("At Target (8 balls)", this::isAtTarget)
            .withPosition(3, 0)
            .withSize(2, 2);

        ballCounterTab.addNumber("Distance (inches)", this::getDistanceInches)
            .withPosition(0, 2)
            .withSize(2, 1);

        ballCounterTab.addBoolean("Ball Detected", this::isBallDetected)
            .withPosition(2, 2)
            .withSize(2, 1);

        System.out.println("==========================================");
        System.out.println("BallCounterSubsystem initialized");
        System.out.println("CANrange CAN ID: " + CANRANGE_ID);
        System.out.println("Detection threshold: " + BALL_DETECTION_THRESHOLD_METERS + " meters (" + Meters.of(BALL_DETECTION_THRESHOLD_METERS).in(Inches) + " inches)");
        System.out.println("Target count: " + TARGET_COUNT + " balls");
        System.out.println("==========================================");
    }

    private void configureCANrange() {
        var cfg = new CANrangeConfiguration();

        // Set proximity threshold for ball detection
        var prox = new ProximityParamsConfigs();
        prox.ProximityThreshold = BALL_DETECTION_THRESHOLD_METERS;
        prox.ProximityHysteresis = 0.02; // 2cm hysteresis to prevent bouncing
        prox.MinSignalStrengthForValidMeasurement = 2000;
        cfg.ProximityParams = prox;

        var status = canRange.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("CANrange config failed: " + status);
        }
    }

    @Override
    public void periodic() {
        // Get current distance and detection state
        double distanceMeters = distanceSignal.refresh().getValue().in(Meters);
        boolean ballDetectedNow = isDetectedSignal.refresh().getValue();

        // Count ball when it first enters the detection zone (rising edge)
        if (ballDetectedNow && !ballWasDetected) {
            ballCount++;
            System.out.println("Ball detected! Count: " + ballCount);
        }
        ballWasDetected = ballDetectedNow;

        // Publish data to SmartDashboard
        SmartDashboard.putNumber("BallCounter/Count", ballCount);
        SmartDashboard.putBoolean("BallCounter/AtTarget", isAtTarget());
        SmartDashboard.putNumber("BallCounter/Distance_m", distanceMeters);
        SmartDashboard.putNumber("BallCounter/Distance_in", Meters.of(distanceMeters).in(Inches));
        SmartDashboard.putBoolean("BallCounter/BallDetected", ballDetectedNow);
        SmartDashboard.putNumber("BallCounter/Threshold_m", BALL_DETECTION_THRESHOLD_METERS);

        // Debug info
        SmartDashboard.putBoolean("BallCounter/Debug_SignalOK", distanceSignal.getStatus().isOK());
        SmartDashboard.putString("BallCounter/Debug_SignalStatus", distanceSignal.getStatus().getName());
        SmartDashboard.putBoolean("BallCounter/CANConnected", distanceSignal.getStatus().isOK());
    }

    /**
     * Gets the current ball count.
     * @return Number of balls counted
     */
    public int getBallCount() {
        return ballCount;
    }

    /**
     * Checks if we've reached the target count of 8 balls.
     * @return true if count >= 8
     */
    public boolean isAtTarget() {
        return ballCount >= TARGET_COUNT;
    }

    /**
     * Gets the current distance reading from the CANrange sensor in inches.
     * @return Distance in inches
     */
    public double getDistanceInches() {
        double distanceMeters = distanceSignal.getValue().in(Meters);
        return Meters.of(distanceMeters).in(Inches);
    }

    /**
     * Checks if a ball is currently detected (within threshold).
     * @return true if ball is detected
     */
    public boolean isBallDetected() {
        return isDetectedSignal.getValue();
    }

    /**
     * Resets the ball count to zero.
     */
    public void resetCount() {
        ballCount = 0;
        System.out.println("Ball count reset to 0");
    }
}
