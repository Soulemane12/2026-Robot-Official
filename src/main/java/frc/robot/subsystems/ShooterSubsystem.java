package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterMotor;
    private boolean m_isRunning = false;

    // Full speed (100% duty cycle)
    private static final double SHOOTER_SPEED = 1.0;

    public ShooterSubsystem(int motorCanId) {
        m_shooterMotor = new TalonFX(motorCanId);

        // Configure motor
        m_shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        // Optional: Set current limit to protect motor
        // var currentLimits = new CurrentLimitsConfigs();
        // currentLimits.StatorCurrentLimit = 80;
        // currentLimits.StatorCurrentLimitEnable = true;
        // m_shooterMotor.getConfigurator().apply(currentLimits);
    }

    @Override
    public void periodic() {
        // Publish shooter state to SmartDashboard
        SmartDashboard.putBoolean("Shooter/IsRunning", m_isRunning);
        SmartDashboard.putNumber("Shooter/MotorOutput", m_shooterMotor.get());
        SmartDashboard.putNumber("Shooter/Velocity", m_shooterMotor.getVelocity().getValueAsDouble());
    }

    /**
     * Toggle the shooter on/off
     */
    public void toggle() {
        if (m_isRunning) {
            stop();
        } else {
            start();
        }
    }

    /**
     * Start the shooter at full speed
     */
    public void start() {
        m_shooterMotor.set(SHOOTER_SPEED);
        m_isRunning = true;
    }

    /**
     * Stop the shooter
     */
    public void stop() {
        m_shooterMotor.set(0);
        m_isRunning = false;
    }

    /**
     * Check if shooter is running
     */
    public boolean isRunning() {
        return m_isRunning;
    }
}
