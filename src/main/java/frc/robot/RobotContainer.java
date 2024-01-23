// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final OI oi = new OI(Constants.OI.driverPort);
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    swerve.setDefaultCommand(swerve.run(() -> swerve.drive(
        Math.pow(MathUtil.applyDeadband(oi.drive_x.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxSpeed
            * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
        Math.pow(MathUtil.applyDeadband(oi.drive_y.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxSpeed
            * (oi.driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1),
        Math.pow(MathUtil.applyDeadband(oi.drive_rot.getAsDouble(), Constants.OI.deadband), 3)
            * Constants.Swerve.maxAngularSpeed * (oi.driveSlow.getAsBoolean() ? 0.15 : 1),
        true,
        true)));

    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::resetGyro));

    oi.launchButton.onTrue(shooter.runOnce(shooter::launchNote));

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
