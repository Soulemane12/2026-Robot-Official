package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterMotor;
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);
    private boolean m_isRunning = false;

    private static final double SHOOTER_VOLTAGE = 10.0; // reduced from 12V — shots were going over
    private static final double AT_SPEED_RPS    = 60.0; // ~3600 RPM — raise if still feeding early

    public ShooterSubsystem(int motorCanId) {
        m_shooterMotor = new TalonFX(motorCanId, frc.robot.generated.TunerConstants.kCANBus);
        m_shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        var tab = Shuffleboard.getTab("Shooting");
        tab.addDouble("Shooter Voltage",  () -> m_shooterMotor.getMotorVoltage().getValueAsDouble()).withPosition(0, 2).withSize(2, 1);
        tab.addDouble("Shooter Velocity", () -> m_shooterMotor.getVelocity().getValueAsDouble())   .withPosition(2, 2).withSize(1, 1);
        tab.addBoolean("Shooter Running", () -> m_isRunning)                                       .withPosition(3, 2).withSize(1, 1);
    }

    @Override
    public void periodic() {
        // Publish shooter state to SmartDashboard
        SmartDashboard.putBoolean("Shooter/IsRunning", m_isRunning);
        SmartDashboard.putBoolean("Shooter/AtSpeed",   isAtSpeed());
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
        m_shooterMotor.setControl(m_voltageOut.withOutput(SHOOTER_VOLTAGE));
        m_isRunning = true;
    }

    /**
     * Set shooter to a specific voltage (used by auto-aim)
     */
    public void setVoltage(double volts) {
        m_shooterMotor.setControl(m_voltageOut.withOutput(volts));
        m_isRunning = volts > 0.0;
    }

    /**
     * Stop the shooter
     */
    public void stop() {
        m_shooterMotor.setControl(m_voltageOut.withOutput(0.0));
        m_isRunning = false;
    }

    public boolean isRunning() {
        return m_isRunning;
    }

    public boolean isAtSpeed() {
        return Math.abs(m_shooterMotor.getVelocity().getValueAsDouble()) >= AT_SPEED_RPS;
    }
}
