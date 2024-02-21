// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Autos autos = new Autos(swerve);

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

    oi.speakerPrepButton.onTrue(shooter.prepSpeakerCommand());
    oi.ampPrepButton.onTrue(shooter.prepAmpCommand());
    oi.fireButton.onTrue(shooter.fireCommand(intake));

    oi.smartIntakeButton.whileTrue(intake.smartIntakeCommand());

    oi.climberUpButton.whileTrue(climber.upCommand());
    oi.climberDownButton.whileTrue(climber.downCommand());

    
    //TODO
    //ampHomeButton
    //ampManualUpButton
    //ampManualDownButton
    oi.manualIntakeButton.whileTrue(intake.run(() -> intake.set(0.2)));
    oi.manualOuttakeButton.whileTrue(intake.run(() -> intake.set(-0.2)));
    oi.climberManualOverrideButton.and(() -> Math.abs(oi.climberLeftSpeed.getAsDouble()) > Constants.OI.deadband)
            .whileTrue(climber.run(() -> climber.setLeftSpeed(
                MathUtil.applyDeadband(oi.climberLeftSpeed.getAsDouble(), Constants.OI.deadband, 1))));
    oi.climberManualOverrideButton.and(() -> Math.abs(oi.climberRightSpeed.getAsDouble()) > Constants.OI.deadband)
            .whileTrue(climber.run(() -> climber.setRightSpeed(
                MathUtil.applyDeadband(oi.climberRightSpeed.getAsDouble(), Constants.OI.deadband, 1))));

  }

  public Command getAutonomousCommand() {
    return autos.getAutonomousCommand();
  }
}
