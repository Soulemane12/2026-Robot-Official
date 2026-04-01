package frc.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    // Track previous game data to detect HUB activation changes
    private String m_previousGameData = "";
    private boolean m_pipelineConfigured = false;

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

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degToMotorRot(Constants.TurretConstants.MAX_DEG);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degToMotorRot(Constants.TurretConstants.MIN_DEG);

        m_motor.getConfigurator().apply(cfg);

        // Boot assumption: turret is physically centered before power-on
        m_motor.setPosition(0.0);
    }

    public void zeroHere() {
        m_motor.setPosition(0.0);
    }

    public double getMotorRot() {
        return m_motor.getPosition().getValueAsDouble();
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

    public void setAngleDeg(double targetDeg) {
        double clamped = clamp(targetDeg, Constants.TurretConstants.MIN_DEG, Constants.TurretConstants.MAX_DEG);
        m_motor.setControl(m_mm.withPosition(degToMotorRot(clamped)));
    }

    public void jogVolts(double volts) {
        double angleDeg = getAngleDeg();
        boolean atMax = volts > 0.0 && angleDeg >= Constants.TurretConstants.MAX_DEG;
        boolean atMin = volts < 0.0 && angleDeg <= Constants.TurretConstants.MIN_DEG;
        m_motor.setControl(m_volts.withOutput((atMax || atMin) ? 0.0 : volts));
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(Constants.VisionConstants.LIMELIGHT_TURRET);
    }

    public double getTxDeg() {
        return LimelightHelpers.getTX(Constants.VisionConstants.LIMELIGHT_TURRET);
    }

    public void configureTrackingPipelineAndFilters() {
        String ll = Constants.VisionConstants.LIMELIGHT_TURRET;
        LimelightHelpers.setPipelineIndex(ll, Constants.VisionConstants.APRILTAG_PIPELINE);

        // Get alliance from dashboard override or DriverStation
        boolean isBlue = SmartDashboard.getBoolean("Turret/ForceBlueAlliance", false);
        boolean isRed = SmartDashboard.getBoolean("Turret/ForceRedAlliance", false);

        Alliance alliance;
        if (isBlue && !isRed) {
            alliance = Alliance.Blue;
        } else if (isRed && !isBlue) {
            alliance = Alliance.Red;
        } else {
            var driverStationAlliance = DriverStation.getAlliance();
            if (driverStationAlliance.isEmpty()) return;
            alliance = driverStationAlliance.get();
        }

        // Always track our own alliance HUB (no switching based on game data)
        int[] ids = alliance == Alliance.Blue
                ? Constants.VisionConstants.BLUE_TRACK_TAG_IDS
                : Constants.VisionConstants.RED_TRACK_TAG_IDS;

        if (ids.length > 0) {
            LimelightHelpers.SetFiducialIDFiltersOverride(ll, ids);
            LimelightHelpers.setPriorityTagID(ll, ids[0]);
        }
    }

    public void aimAtAprilTag() {
        if (!hasTarget()) return;

        double tx = getTxDeg();

        // Use dashboard tuning values
        double deadband = SmartDashboard.getNumber("Turret/TxDeadband", Constants.VisionConstants.TX_DEADBAND_DEG);
        double offset = SmartDashboard.getNumber("Turret/TrackingOffset", Constants.VisionConstants.TRACKING_ZERO_OFFSET_DEG);

        if (Math.abs(tx) < deadband) tx = 0.0;

        double desiredDeg = offset + Constants.VisionConstants.TX_TO_TURRET_SIGN * tx;

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
        SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("Turret/MotorRot", getMotorRot());
        SmartDashboard.putNumber("Turret/VelocityRPS", m_motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret/SupplyCurrentA", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Turret/HasTarget", hasTarget());
        SmartDashboard.putNumber("Turret/TxDeg", getTxDeg());

        // Auto-reconfigure pipeline when alliance override changes
        boolean forceBlue = SmartDashboard.getBoolean("Turret/ForceBlueAlliance", false);
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
