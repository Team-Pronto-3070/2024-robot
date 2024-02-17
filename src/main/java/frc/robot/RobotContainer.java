// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  private final OI oi = new OI(Constants.OI.driverPort);
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    swerve.setDefaultCommand(swerve.run(() -> swerve.drive(
        oi.processed_drive_x.getAsDouble(),
        oi.processed_drive_y.getAsDouble(),
        oi.processed_drive_rot.getAsDouble(),
        true,
        true)));
    shooter.setDefaultCommand(shooter.run(shooter::stop));
    intake.setDefaultCommand(intake.run(intake::stop));
    climber.setDefaultCommand(climber.run(climber::stop));

    oi.interruptButton.onTrue(swerve.runOnce(swerve::stop))
                      .onTrue(intake.runOnce(intake::stop))
                      .onTrue(shooter.runOnce(shooter::stop))
                      .onTrue(climber.runOnce(climber::stop));

    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::resetGyro));

    oi.speakerLaunchButton.onTrue(shooter.runOnce(shooter::launchSpeakerNote));
    oi.ampLaunchButton.onTrue(shooter.runOnce(shooter::launchAmpNote));

    oi.smartIntakeButton.onTrue(intake.smartIntake());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
