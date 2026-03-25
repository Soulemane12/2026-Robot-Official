package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_motor;

    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0).withSlot(0);
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    private double m_targetDeg = 0.0;

    public TurretSubsystem() {
        m_motor = new TalonFX(Constants.CANIds.TURRET_MOTOR, frc.robot.generated.TunerConstants.kCANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode so turret holds position when idle
        m_motor.setNeutralMode(NeutralModeValue.Brake);

        // PID/FF — tune on robot
        Slot0Configs slot0 = new Slot0Configs()
            .withKP(30.0)
            .withKD(0.2)
            .withKS(0.4)
            .withKV(0.12);
        config.Slot0 = slot0;

        // Motion Magic profile — tune on robot
        MotionMagicConfigs mmConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(6.0)   // motor RPS
            .withMotionMagicAcceleration(15.0)
            .withMotionMagicJerk(150.0);
        config.MotionMagic = mmConfig;

        // Current limits
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 50;

        // Software soft limits (hardware-enforced in the Talon)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degToMotorRot(Constants.TurretConstants.MAX_DEG);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degToMotorRot(Constants.TurretConstants.MIN_DEG);

        m_motor.getConfigurator().apply(config);

        // Zero encoder — user physically places turret at center before boot
        m_motor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("Turret/TargetDeg", m_targetDeg);
        SmartDashboard.putNumber("Turret/VelocityRPS", m_motor.getVelocity().getValueAsDouble());
    }

    /** Command turret to an angle in degrees relative to center zero. */
    public void setAngleDeg(double deg) {
        m_targetDeg = clamp(deg, Constants.TurretConstants.MIN_DEG, Constants.TurretConstants.MAX_DEG);
        m_motor.setControl(m_motionMagic.withPosition(degToMotorRot(m_targetDeg)));
    }

    /** Manual voltage jog with soft limit guards. */
    public void jogVolts(double volts) {
        double pos = getAngleDeg();
        if (volts > 0.0 && pos >= Constants.TurretConstants.MAX_DEG) {
            stop();
            return;
        }
        if (volts < 0.0 && pos <= Constants.TurretConstants.MIN_DEG) {
            stop();
            return;
        }
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    public void stop() {
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    /** Re-zero the encoder at the current physical position. */
    public void zeroHere() {
        m_motor.setPosition(0.0);
        m_targetDeg = 0.0;
    }

    public double getAngleDeg() {
        return motorRotToDeg(m_motor.getPosition().getValueAsDouble());
    }

    // ===== Conversions =====

    private static double degToMotorRot(double deg) {
        return (deg / 360.0) * Constants.TurretConstants.GEAR_RATIO;
    }

    private static double motorRotToDeg(double motorRot) {
        return (motorRot / Constants.TurretConstants.GEAR_RATIO) * 360.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
