package frc.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_motor =
            new TalonFX(Constants.CANIds.TURRET_MOTOR, TunerConstants.kCANBus);

    private final MotionMagicVoltage m_mm = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut m_volts = new VoltageOut(0);

    private String m_previousGameData = "";
    private boolean m_pipelineConfigured = false;
    private boolean m_ferryMode = false; // false = normal (mid-field tags), true = ferry (alliance-zone tags)
    private int[] m_allowedTagIds = Constants.VisionConstants.BLUE_HUB_TAG_IDS; // updated by configureTrackingPipelineAndFilters
    private double m_cachedRotations = 0.0;
    private int m_lockedTagId = -1; // -1 = no lock; set on first valid target

    public TurretSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.STATOR_LIMIT_A;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.SUPPLY_LIMIT_A;
        cfg.CurrentLimits.SupplyCurrentLowerLimit  = Constants.TurretConstants.SUPPLY_THRESHOLD_A;
        cfg.CurrentLimits.SupplyCurrentLowerTime   = Constants.TurretConstants.SUPPLY_TIME_S;

        cfg.Slot0.kP = Constants.TurretConstants.kP;
        cfg.Slot0.kD = Constants.TurretConstants.kD;
        cfg.Slot0.kS = Constants.TurretConstants.kS;
        cfg.Slot0.kV = Constants.TurretConstants.kV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.CRUISE_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = Constants.TurretConstants.ACCEL_RPS2;
        cfg.MotionMagic.MotionMagicJerk           = Constants.TurretConstants.JERK_RPS3;

        cfg.ClosedLoopGeneral.ContinuousWrap = false;

        m_motor.getConfigurator().apply(cfg);

        // Boot assumption: turret is physically centered before power-on
        m_motor.setPosition(0.0);

        SmartDashboard.putBoolean("Turret/ForceBlueAlliance", true);
        SmartDashboard.putBoolean("Turret/ForceRedAlliance", false);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooting");
        tab.addDouble("Turret Angle (deg)", this::getAngleDeg)   .withPosition(0, 0).withSize(2, 1);
        tab.addDouble("TX (deg)",           this::getTxDeg)       .withPosition(2, 0).withSize(1, 1);
        tab.addBoolean("Has Target",        this::hasTarget)      .withPosition(3, 0).withSize(1, 1);
        tab.addDouble("Tag ID",             () -> LimelightHelpers.getFiducialID(Constants.VisionConstants.LIMELIGHT_TURRET))
                                                                   .withPosition(4, 0).withSize(1, 1);
        tab.addDouble("Distance (m)",       this::getDistanceToTargetM).withPosition(5, 0).withSize(1, 1);
        tab.addBoolean("On Target",         () -> SmartDashboard.getBoolean("AutoAim/OnTarget",     false)).withPosition(3, 1).withSize(1, 1);
        tab.addBoolean("Ready To Shoot",    () -> SmartDashboard.getBoolean("AutoAim/ReadyToShoot", false)).withPosition(4, 1).withSize(2, 1);
    }

    public void zeroHere() {
        m_motor.setPosition(0.0);
    }

    public double getMotorRot() {
        return m_cachedRotations;
    }

    public double getAngleDeg() {
        return motorRotToDeg(getMotorRot());
    }

    public boolean atAngle(double targetDeg) {
        return Math.abs(getAngleDeg() - targetDeg) <= Constants.TurretConstants.ANGLE_TOLERANCE_DEG;
    }

    public void stop() {
        m_motor.stopMotor();
    }

    // Angle range blocked by a physical obstruction — turret cannot see through here
    private static final double BLOCKED_MIN_DEG = -77.0;
    private static final double BLOCKED_MAX_DEG = -32.0;

    /** Redirects a target angle away from the blocked zone to the nearest clear boundary. */
    private double avoidBlockedZone(double targetDeg) {
        if (targetDeg >= BLOCKED_MIN_DEG && targetDeg <= BLOCKED_MAX_DEG) {
            double currentDeg = getAngleDeg();
            if (currentDeg > BLOCKED_MAX_DEG) return BLOCKED_MAX_DEG; // stay on upper side
            if (currentDeg < BLOCKED_MIN_DEG) return BLOCKED_MIN_DEG; // stay on lower side
            // Current is also inside — snap to nearest boundary
            return (Math.abs(targetDeg - BLOCKED_MIN_DEG) < Math.abs(targetDeg - BLOCKED_MAX_DEG))
                ? BLOCKED_MIN_DEG : BLOCKED_MAX_DEG;
        }
        return targetDeg;
    }

    public void setAngleDeg(double targetDeg) {
        double safe    = avoidBlockedZone(targetDeg);
        double clamped = clamp(safe, Constants.TurretConstants.MIN_DEG, Constants.TurretConstants.MAX_DEG);
        m_motor.setControl(m_mm.withPosition(degToMotorRot(clamped)));
    }

    public void jogVolts(double volts) {
        m_motor.setControl(m_volts.withOutput(volts));
    }

    public void toggleFerryMode() {
        m_ferryMode = !m_ferryMode;
        configureTrackingPipelineAndFilters(); // apply immediately
    }

    public boolean isFerryMode() { return m_ferryMode; }

    public void setFerryMode(boolean ferryMode) {
        m_ferryMode = ferryMode;
    }

    public int getLockedTagId() { return m_lockedTagId; }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(Constants.VisionConstants.LIMELIGHT_TURRET);
    }

    public double getTxDeg() {
        return LimelightHelpers.getTX(Constants.VisionConstants.LIMELIGHT_TURRET);
    }

    /** Horizontal ground-plane distance to the tracked target in meters. Returns -1 if no target or bad data. */
    public double getDistanceToTargetM() {
        if (!hasTarget()) return -1.0;
        // Use CameraSpace pose — always valid when TV=1, no Limelight transform config required.
        double[] pose = LimelightHelpers.getTargetPose_CameraSpace(Constants.VisionConstants.LIMELIGHT_TURRET);
        if (pose == null || pose.length < 3) return -1.0;
        double x = pose[0]; // left/right in camera frame
        double z = pose[2]; // forward in camera frame
        double dist = Math.sqrt(x * x + z * z);
        return dist > 0.1 ? dist : -1.0; // reject all-zero garbage from no-pose state
    }

    public void configureTrackingPipelineAndFilters() {
        String ll = Constants.VisionConstants.LIMELIGHT_TURRET;
        LimelightHelpers.setPipelineIndex(ll, Constants.VisionConstants.APRILTAG_PIPELINE);
        LimelightHelpers.setLEDMode_ForceOn(ll);

        m_lockedTagId = -1; // reset tag lock when pipeline is (re-)configured

        // Do NOT use SetFiducialIDFiltersOverride — it blocks tv=1 on some firmware versions.
        // Instead, filter by tag ID in software inside aimAtAprilTag().
        boolean isBlue = SmartDashboard.getBoolean("Turret/ForceBlueAlliance", true);
        boolean isRed  = SmartDashboard.getBoolean("Turret/ForceRedAlliance", false);

        Alliance alliance;
        if (isBlue && !isRed) {
            alliance = Alliance.Blue;
        } else if (isRed && !isBlue) {
            alliance = Alliance.Red;
        } else {
            var ds = DriverStation.getAlliance();
            if (ds.isEmpty()) return;
            alliance = ds.get();
        }

        if (alliance == Alliance.Blue) {
            m_allowedTagIds = m_ferryMode ? Constants.VisionConstants.BLUE_FERRY_TAG_IDS
                                          : Constants.VisionConstants.BLUE_TRACK_TAG_IDS;
        } else {
            m_allowedTagIds = m_ferryMode ? Constants.VisionConstants.RED_FERRY_TAG_IDS
                                          : Constants.VisionConstants.RED_TRACK_TAG_IDS;
        }
    }

    /**
     * Gets the ferry-specific turret offset for the currently locked AprilTag.
     * @return Offset in degrees (positive = right, negative = left), or 0.0 if no tag locked
     */
    public double getFerryOffsetForCurrentTag() {
        if (m_lockedTagId < 0) return 0.0;
        return Constants.VisionConstants.FERRY_TAG_OFFSETS.getOrDefault(m_lockedTagId, 0.0);
    }

    public void aimAtAprilTag() {
        aimAtAprilTag(false, 0.0);
    }

    public void aimAtAprilTag(boolean useFerryOffset) {
        aimAtAprilTag(useFerryOffset, 0.0);
    }

    public void aimAtAprilTag(boolean useFerryOffset, double leadDeg) {
        if (!hasTarget()) {
            SmartDashboard.putBoolean("Turret/AimHasTarget", false);
            m_lockedTagId = -1;
            return;
        }

        int tagId = (int) LimelightHelpers.getFiducialID(Constants.VisionConstants.LIMELIGHT_TURRET);

        // Ferry mode: only track alliance-specific tags; normal mode: track any visible tag
        if (useFerryOffset) {
            boolean tagAllowed = Arrays.stream(m_allowedTagIds).anyMatch(id -> id == tagId);
            if (!tagAllowed) {
                SmartDashboard.putBoolean("Turret/AimHasTarget", false);
                m_lockedTagId = -1;
                return;
            }
        }

        // Lock onto the first valid tag seen; ignore other tags while locked.
        // This prevents jumping between multiple visible hub tags.
        if (m_lockedTagId == -1) {
            m_lockedTagId = tagId;
        } else if (tagId != m_lockedTagId) {
            // Different tag visible — keep commanding toward current angle (hold position)
            SmartDashboard.putBoolean("Turret/AimHasTarget", false);
            return;
        }

        double tx = getTxDeg();

        // Use dashboard tuning values
        double deadband = SmartDashboard.getNumber("Turret/TxDeadband", Constants.VisionConstants.TX_DEADBAND_DEG);
        double offset = useFerryOffset
            ? getFerryOffsetForCurrentTag()
            : SmartDashboard.getNumber("Turret/TrackingOffset", Constants.VisionConstants.TRACKING_ZERO_OFFSET_DEG);

        if (Math.abs(tx) < deadband) tx = 0.0;

        double desiredDeg = getAngleDeg() + offset + Constants.VisionConstants.TX_TO_TURRET_SIGN * tx + leadDeg;

        SmartDashboard.putBoolean("Turret/AimHasTarget", true);
        SmartDashboard.putNumber("Turret/AimTargetDeg", desiredDeg);
        setAngleDeg(desiredDeg);
    }

    public Command trackAprilTagCommand() {
        return run(this::aimAtAprilTag)
                .beforeStarting(this::configureTrackingPipelineAndFilters);
    }

    private static double degToMotorRot(double turretDeg) {
        return (turretDeg / 360.0) * Constants.TurretConstants.GEAR_RATIO;
    }

    private static double motorRotToDeg(double motorRot) {
        return (motorRot / Constants.TurretConstants.GEAR_RATIO) * 360.0;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        // CANivore signals auto-update asynchronously — no refreshAll() needed
        m_cachedRotations = m_motor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("Turret/MotorRot", getMotorRot());
        SmartDashboard.putNumber("Turret/VelocityRPS", m_motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret/SupplyCurrentA", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Turret/HasTarget", hasTarget());
        SmartDashboard.putNumber("Turret/TxDeg", getTxDeg());
        SmartDashboard.putString("Turret/LimelightName", Constants.VisionConstants.LIMELIGHT_TURRET);
        SmartDashboard.putNumber("Turret/Pipeline", LimelightHelpers.getCurrentPipelineIndex(Constants.VisionConstants.LIMELIGHT_TURRET));
        SmartDashboard.putNumber("Turret/TagId", LimelightHelpers.getFiducialID(Constants.VisionConstants.LIMELIGHT_TURRET));

        // Auto-reconfigure pipeline when alliance override changes
        boolean forceBlue = SmartDashboard.getBoolean("Turret/ForceBlueAlliance", true);
        boolean forceRed = SmartDashboard.getBoolean("Turret/ForceRedAlliance", false);
        String allianceKey = forceBlue + "_" + forceRed;

        if (!allianceKey.equals(m_previousGameData) || !m_pipelineConfigured) {
            configureTrackingPipelineAndFilters();
            m_previousGameData = allianceKey;
            m_pipelineConfigured = true;
        }

        // Dashboard controls and info
        SmartDashboard.putBoolean("Turret/ForceBlueAlliance", forceBlue);
        SmartDashboard.putBoolean("Turret/ForceRedAlliance", forceRed);

        // Vision tuning controls
        SmartDashboard.putNumber("Turret/TxDeadband",
            SmartDashboard.getNumber("Turret/TxDeadband", Constants.VisionConstants.TX_DEADBAND_DEG));
        SmartDashboard.putNumber("Turret/TrackingOffset",
            SmartDashboard.getNumber("Turret/TrackingOffset", Constants.VisionConstants.TRACKING_ZERO_OFFSET_DEG));

        // Current alliance status
        var dsAlliance = DriverStation.getAlliance();
        String currentAlliance = "Unknown";
        if (forceBlue && !forceRed) currentAlliance = "Blue (Forced)";
        else if (forceRed && !forceBlue) currentAlliance = "Red (Forced)";
        else if (dsAlliance.isPresent()) currentAlliance = dsAlliance.get().toString() + " (DS)";

        SmartDashboard.putString("Turret/CurrentAlliance", currentAlliance);
    }
}
