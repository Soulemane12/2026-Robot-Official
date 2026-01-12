package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command that aligns the robot to an AprilTag using PID control.
 * The robot will:
 * - Position itself at a target distance from the AprilTag (X-axis control)
 * - Center itself horizontally with the AprilTag (Y-axis control)
 * - Rotate to face the AprilTag head-on (rotation control)
 *
 * This command finishes when the robot is aligned within tolerances for a set duration,
 * or when the tag is lost for too long.
 */
public class AlignToAprilTag extends Command {
    private final VisionSubsystem m_vision;
    private final CommandSwerveDrivetrain m_swerve;
    private final SwerveRequest.RobotCentric m_driveRequest;

    // PID Controllers
    private PIDController xController;  // Forward/backward distance control
    private PIDController yController;  // Left/right centering control
    private PIDController rotController;  // Rotation control to face tag

    // Timers
    private Timer dontSeeTagTimer;  // Timer for when tag is not visible
    private Timer stopTimer;  // Timer to verify robot stays aligned

    // Target tag ID (locks onto first tag seen)
    private double targetTagID = -1;

    private int debugCounter = 0;

    /**
     * Creates a new AlignToAprilTag command.
     * Uses constants from Constants.java for PID gains, setpoints, and tolerances.
     *
     * @param vision The vision subsystem
     * @param swerve The swerve drivetrain
     */
    public AlignToAprilTag(VisionSubsystem vision, CommandSwerveDrivetrain swerve) {
        m_vision = vision;
        m_swerve = swerve;

        // Initialize PID controllers with gains from Constants
        xController = new PIDController(Constants.X_APRILTAG_ALIGNMENT_P, 0.0, 0.0);
        yController = new PIDController(Constants.Y_APRILTAG_ALIGNMENT_P, 0.0, 0.0);
        rotController = new PIDController(Constants.ROT_APRILTAG_ALIGNMENT_P, 0.0, 0.0);

        // Set up the drive request for robot-centric control
        m_driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.0)  // No deadband for precise alignment
            .withRotationalDeadband(0.0);

        addRequirements(m_vision, m_swerve);
    }

    @Override
    public void initialize() {
        System.out.println("===========================================");
        System.out.println("AlignToAprilTag started - looking for AprilTag...");
        System.out.println("Using Limelight: \"" + m_vision.getLimelightName() + "\"");
        System.out.println("===========================================");

        // Initialize timers
        dontSeeTagTimer = new Timer();
        dontSeeTagTimer.start();

        stopTimer = new Timer();
        stopTimer.start();

        // Configure rotation controller
        rotController.setSetpoint(Constants.ROT_SETPOINT_APRILTAG_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_APRILTAG_ALIGNMENT);

        // Configure X controller (distance)
        xController.setSetpoint(Constants.X_SETPOINT_APRILTAG_ALIGNMENT);
        xController.setTolerance(Constants.X_TOLERANCE_APRILTAG_ALIGNMENT);

        // Configure Y controller (centering)
        yController.setSetpoint(Constants.Y_SETPOINT_APRILTAG_ALIGNMENT);
        yController.setTolerance(Constants.Y_TOLERANCE_APRILTAG_ALIGNMENT);

        // Lock onto the first tag we see
        targetTagID = m_vision.getTagID();

        debugCounter = 0;

        System.out.println("Target Tag ID: " + targetTagID);
        System.out.println("Target X (distance): " + Constants.X_SETPOINT_APRILTAG_ALIGNMENT + "m");
        System.out.println("Target Y (centering): " + Constants.Y_SETPOINT_APRILTAG_ALIGNMENT + "m");
        System.out.println("Target Rotation: " + Constants.ROT_SETPOINT_APRILTAG_ALIGNMENT + " degrees");
    }

    @Override
    public void execute() {
        boolean hasTarget = m_vision.hasValidTarget();
        double currentTagID = m_vision.getTagID();

        // Check if we can see the correct tag
        if (hasTarget && currentTagID == targetTagID) {
            // Reset the "don't see tag" timer since we see it
            dontSeeTagTimer.reset();

            // Get 3D position relative to the AprilTag
            // positions[0] = X (left/right from tag's perspective)
            // positions[1] = Y (up/down)
            // positions[2] = Z (distance forward/back)
            // positions[3] = roll
            // positions[4] = yaw (rotation)
            // positions[5] = pitch
            double[] positions = LimelightHelpers.getBotPose_TargetSpace(m_vision.getLimelightName());

            // Calculate control outputs using PID
            // Note: positions[2] is Z-distance (forward), positions[0] is X (strafe)
            double xSpeed = xController.calculate(positions[2]);  // Distance control
            double ySpeed = -yController.calculate(positions[0]);  // Centering control (negated for correct direction)
            double rotValue = -rotController.calculate(positions[4]);  // Rotation control (negated for correct direction)

            // Apply control to the drivetrain
            m_swerve.setControl(
                m_driveRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotValue)
            );

            // Check if all controllers are at their setpoints
            if (!rotController.atSetpoint() ||
                !yController.atSetpoint() ||
                !xController.atSetpoint()) {
                // Not aligned yet, reset the stop timer
                stopTimer.reset();
            }

            // Debug output
            debugCounter++;
            if (debugCounter >= 10) {
                debugCounter = 0;
                System.out.println("--- AlignToAprilTag Debug ---");
                System.out.println("  Position - X: " + String.format("%.2f", positions[0]) +
                                 " Y: " + String.format("%.2f", positions[1]) +
                                 " Z: " + String.format("%.2f", positions[2]));
                System.out.println("  Rotation (Yaw): " + String.format("%.2f", positions[4]));
                System.out.println("  At Setpoint - X: " + xController.atSetpoint() +
                                 " Y: " + yController.atSetpoint() +
                                 " Rot: " + rotController.atSetpoint());
                System.out.println("  Stop Timer: " + String.format("%.2f", stopTimer.get()) + "s");
            }

            // Publish data to SmartDashboard
            SmartDashboard.putNumber("AprilTag/Position_X", positions[0]);
            SmartDashboard.putNumber("AprilTag/Position_Y", positions[1]);
            SmartDashboard.putNumber("AprilTag/Position_Z", positions[2]);
            SmartDashboard.putNumber("AprilTag/Rotation_Yaw", positions[4]);
            SmartDashboard.putNumber("AprilTag/XSpeed", xSpeed);
            SmartDashboard.putNumber("AprilTag/YSpeed", ySpeed);
            SmartDashboard.putNumber("AprilTag/RotValue", rotValue);
            SmartDashboard.putBoolean("AprilTag/X_AtSetpoint", xController.atSetpoint());
            SmartDashboard.putBoolean("AprilTag/Y_AtSetpoint", yController.atSetpoint());
            SmartDashboard.putBoolean("AprilTag/Rot_AtSetpoint", rotController.atSetpoint());
            SmartDashboard.putNumber("AprilTag/StopTimer", stopTimer.get());

        } else {
            // No target visible or wrong tag - stop moving
            m_swerve.setControl(
                m_driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );

            System.out.println("WARNING: Lost target or wrong tag! Looking for tag ID: " + targetTagID);
        }

        SmartDashboard.putBoolean("AprilTag/HasTarget", hasTarget);
        SmartDashboard.putNumber("AprilTag/CurrentTagID", currentTagID);
        SmartDashboard.putNumber("AprilTag/TargetTagID", targetTagID);
        SmartDashboard.putNumber("AprilTag/DontSeeTagTimer", dontSeeTagTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("===========================================");
        System.out.println("AlignToAprilTag ended " + (interrupted ? "(interrupted)" : "(completed)"));
        System.out.println("===========================================");

        // Stop the robot
        m_swerve.setControl(
            m_driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );

        dontSeeTagTimer.stop();
        stopTimer.stop();
    }

    @Override
    public boolean isFinished() {
        // Command finishes when:
        // 1. Robot doesn't see the tag for too long (timeout)
        // 2. Robot is aligned within tolerances for the required duration
        boolean tagLost = dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME);
        boolean aligned = stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);

        if (tagLost) {
            System.out.println("Command finishing: Tag lost for too long");
        } else if (aligned) {
            System.out.println("Command finishing: Robot aligned!");
        }

        return tagLost || aligned;
    }
}
