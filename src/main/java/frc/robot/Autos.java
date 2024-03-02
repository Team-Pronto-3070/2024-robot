package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AmpBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final SendableChooser<Command> autoChooser;
    private final SwerveSubsystem swerve;

    public Autos(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake, AmpBarSubsystem ampBar) {
        this.swerve = swerve;

        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getChassisSpeeds,
            swerve::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                Constants.Autos.translationPID,
                Constants.Autos.rotationPID,
                Constants.Autos.maxModuleSpeed,
                Constants.Autos.driveBaseRadius,
                new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                } return false;
            },
            swerve
        );

        NamedCommands.registerCommand("intake",
                Commands.sequence(
                    new InstantCommand(() -> SmartDashboard.putBoolean("intake command running", true)),
                    intake.run(() -> intake.set(1)).until(new Trigger(() -> intake.hasNote())),
                    Commands.defer(() -> intake.run(() -> intake.set(-0.1)).until(new Trigger(() -> intake.hasNote()).debounce(0.04)), Set.of(intake)),
                    Commands.defer(() -> intake.run(() -> intake.set(-0.12)).until(new Trigger(() -> !intake.hasNote()).debounce(0.08)), Set.of(intake)),
                    Commands.defer(() -> intake.run(() -> intake.set(0.1)).until(new Trigger(() -> intake.hasNote()).debounce(0.04)), Set.of(intake)),
                    intake.runOnce(intake::stop)
                ).finallyDo(() -> SmartDashboard.putBoolean("intake command running", false))
        );

        NamedCommands.registerCommand("prepShooter", shooter.prepSpeakerCommandOnce());

        NamedCommands.registerCommand("launchNote", Commands.sequence(
                                    shooter.prepSpeakerCommandOnce(),
                                    Commands.sequence(
                                        Commands.defer(() -> Commands.waitUntil(new Trigger(shooter::atTarget).debounce(0.2)), Set.of()),
                                        intake.run(() -> intake.set(0.2)).withTimeout(0.2),
                                        intake.runOnce(intake::stop),
                                        shooter.runOnce(shooter::stop),
                                        Commands.either(ampBar.homeCommand(), Commands.none(), () -> shooter.target == ShooterSubsystem.Target.AMP),
                                        new InstantCommand(() -> shooter.target = ShooterSubsystem.Target.NONE)
                                    )
                                ));

        NamedCommands.registerCommand("stopIntake", intake.runOnce(intake::stop));
        NamedCommands.registerCommand("stopShooter", shooter.runOnce(shooter::stop));

        NamedCommands.registerCommand("defensiveIntake", intake.run(() -> intake.set(1)));
        NamedCommands.registerCommand("defensiveShooter", shooter.run(() -> shooter.set(300)));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected()
            .andThen(new InstantCommand(() -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    if (alliance.get() == DriverStation.Alliance.Red) {
                        swerve.resetOdometry(swerve.getPose().rotateBy(Rotation2d.fromDegrees(180)));
                    }
                }
            }));
    }

}
