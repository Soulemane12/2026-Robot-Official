// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.FuelSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final HttpCamera m_turretLimelightCamera;
  private final FuelSim m_fuelSim = new FuelSim();

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_turretLimelightCamera =
        new HttpCamera(
            "Limelight Turret",
            "http://" + Constants.VisionConstants.LIMELIGHT_TURRET + ".local:5800/stream.mjpg");
    CameraServer.startAutomaticCapture(m_turretLimelightCamera);

    // Forward the turret Limelight web ports through the roboRIO for diagnostics.
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, Constants.VisionConstants.LIMELIGHT_TURRET + ".local", port);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Battery/VoltageV", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Battery/CurrentA", RobotController.getInputCurrent());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    // Register the robot dimensions and pose suppliers with FuelSim
    // Robot is approximately 30" x 30" (0.762m x 0.762m) based on module positions
    m_fuelSim.registerRobot(
        Meters.of(0.762),  // width
        Meters.of(0.762),  // length
        Meters.of(0.25),   // bumper height
        () -> m_robotContainer.drivetrain.getState().Pose,
        () -> m_robotContainer.drivetrain.getState().Speeds
    );

    // Spawn starting fuel pieces on the field
    m_fuelSim.spawnStartingFuel();

    // Start the simulation
    m_fuelSim.start();
  }

  @Override
  public void simulationPeriodic() {
    m_fuelSim.updateSim();
    m_robotContainer.updateShooterSim(m_fuelSim);
  }
}
