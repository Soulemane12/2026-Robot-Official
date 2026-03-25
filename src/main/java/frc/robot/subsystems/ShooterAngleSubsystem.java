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

public class ShooterAngleSubsystem extends SubsystemBase {
    private final TalonFX m_motor;

    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0).withSlot(0);
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    private double m_targetDeg = Constants.ShooterAngleConstants.MIN_DEG;

    public ShooterAngleSubsystem() {
        m_motor = new TalonFX(Constants.CANIds.SHOOTER_ANGLE_MOTOR, frc.robot.generated.TunerConstants.kCANBus);

        // Brake so it holds position against gravity
        m_motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID / FF
        Slot0Configs slot0 = new Slot0Configs()
            .withKP(Constants.ShooterAngleConstants.kP)
            .withKD(Constants.ShooterAngleConstants.kD)
            .withKS(Constants.ShooterAngleConstants.kS)
            .withKV(Constants.ShooterAngleConstants.kV);
        config.Slot0 = slot0;

        // Motion Magic profile
        MotionMagicConfigs mmConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.ShooterAngleConstants.CRUISE_RPS)
            .withMotionMagicAcceleration(Constants.ShooterAngleConstants.ACCEL_RPS2)
            .withMotionMagicJerk(Constants.ShooterAngleConstants.JERK_RPS3);
        config.MotionMagic = mmConfig;

        // Current limits
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 50;

        // Software soft limits (hardware-enforced in the Talon)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            degToMotorRot(Constants.ShooterAngleConstants.MAX_DEG);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            degToMotorRot(Constants.ShooterAngleConstants.MIN_DEG);

        m_motor.getConfigurator().apply(config);

        // User places hood at MIN_DEG before boot — zero encoder here
        m_motor.setPosition(degToMotorRot(Constants.ShooterAngleConstants.MIN_DEG));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterAngle/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("ShooterAngle/TargetDeg", m_targetDeg);
        SmartDashboard.putNumber("ShooterAngle/VelocityRPS", m_motor.getVelocity().getValueAsDouble());
    }

    /** Command the hood to a specific angle in degrees. */
    public void setAngleDeg(double deg) {
        m_targetDeg = clamp(deg, Constants.ShooterAngleConstants.MIN_DEG, Constants.ShooterAngleConstants.MAX_DEG);
        m_motor.setControl(m_motionMagic.withPosition(degToMotorRot(m_targetDeg)));
    }

    /** Manual voltage jog with soft limit guards. */
    public void jogVolts(double volts) {
        double angle = getAngleDeg();
        if (volts > 0.0 && angle >= Constants.ShooterAngleConstants.MAX_DEG) {
            stop();
            return;
        }
        if (volts < 0.0 && angle <= Constants.ShooterAngleConstants.MIN_DEG) {
            stop();
            return;
        }
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    public void stop() {
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    /** Re-zero the encoder if needed (call when hood is physically at MIN_DEG). */
    public void zeroHere() {
        m_motor.setPosition(degToMotorRot(Constants.ShooterAngleConstants.MIN_DEG));
        m_targetDeg = Constants.ShooterAngleConstants.MIN_DEG;
    }

    public double getAngleDeg() {
        return motorRotToDeg(m_motor.getPosition().getValueAsDouble());
    }

    // ===== Two-point calibration conversions =====

    private static double degToMotorRot(double deg) {
        return Constants.ShooterAngleConstants.M_ROT_PER_DEG * deg
             + Constants.ShooterAngleConstants.B_ROT;
    }

    private static double motorRotToDeg(double rot) {
        return (rot - Constants.ShooterAngleConstants.B_ROT)
             / Constants.ShooterAngleConstants.M_ROT_PER_DEG;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
