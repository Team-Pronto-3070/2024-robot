// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AmpBarSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final OI oi = new OI();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final AmpBarSubsystem ampBar = new AmpBarSubsystem();

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
    //ampBar.setDefaultCommand(ampBar.run(ampBar::stop));

    oi.interruptButton.onTrue(swerve.runOnce(swerve::stop))
                      .onTrue(intake.runOnce(intake::stop))
                      .onTrue(shooter.runOnce(shooter::stop))
                      .onTrue(climber.runOnce(climber::stop))
                      .onTrue(ampBar.runOnce(ampBar::stop));

    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::resetGyro));

    oi.speakerPrepButton.onTrue(shooter.prepSpeakerCommand());
    oi.ampPrepButton.onTrue(shooter.prepAmpCommand(ampBar));
    oi.fireButton.onTrue(shooter.fireCommand(intake, ampBar));

    oi.smartIntakeButton.whileTrue(intake.smartIntakeCommand2());

    oi.climberUpButton.whileTrue(climber.upCommand());
    oi.climberDownButton.onTrue(climber.downCommand());

    
    oi.ampHomeButton.onTrue(ampBar.homeCommand());
    oi.ampUpButton.onTrue(ampBar.upCommand());
    oi.ampManualDownButton.whileTrue(ampBar.run(() -> ampBar.set(0.4)));
    oi.ampManualUpButton.whileTrue(ampBar.run(() -> ampBar.set(-0.4)));
    oi.manualIntakeButton.whileTrue(intake.run(() -> intake.set(1)));
    oi.manualOuttakeButton.whileTrue(intake.run(() -> intake.set(-0.2)));
    oi.climberManualOverrideButton.and(() -> Math.abs(oi.climberLeftSpeed.getAsDouble()) > Constants.OI.deadband)
            .whileTrue(climber.run(() -> climber.setLeftSpeed(
                MathUtil.applyDeadband(oi.climberLeftSpeed.getAsDouble(), Constants.OI.deadband, 1))));
    oi.climberManualOverrideButton.and(() -> Math.abs(oi.climberRightSpeed.getAsDouble()) > Constants.OI.deadband)
            .whileTrue(climber.run(() -> climber.setRightSpeed(
                MathUtil.applyDeadband(oi.climberRightSpeed.getAsDouble(), Constants.OI.deadband, 1))));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
