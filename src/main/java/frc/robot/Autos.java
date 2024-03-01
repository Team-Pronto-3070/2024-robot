package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AmpBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake, AmpBarSubsystem ampBar) {
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

        NamedCommands.registerCommand("intake", intake.autoSmartIntakeCommand2());

        NamedCommands.registerCommand("prepShooter", shooter.prepSpeakerCommandOnce());
        NamedCommands.registerCommand("launchNote", Commands.sequence(
                                    shooter.prepSpeakerCommandOnce(),
                                    shooter.autoFireCommand(intake, ampBar)
                                ));
        //NamedCommands.registerCommand("launchNoteContinuous", Commands.sequence(
        //                            shooter.prepSpeakerCommandOnce(),
        //                            shooter.fireContinuous(intake)
        //                        ));

       NamedCommands.registerCommand("stopIntake", intake.run(intake::stop));
       NamedCommands.registerCommand("stopShooter", shooter.run(shooter::stop));

       NamedCommands.registerCommand("defensiveIntake", Commands.run(() -> intake.set(1)));
       NamedCommands.registerCommand("defensiveShooter", Commands.run(() -> shooter.set(300)));

        // ...

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

    
}
