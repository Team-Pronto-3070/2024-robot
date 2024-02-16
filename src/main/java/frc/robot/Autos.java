package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem swerve) {
        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getChassisSpeeds,
            swerve::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                Constants.Autos.translationPID,
                Constants.Autos.rotationPID,
                Constants.Autos.maxSpeed,
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

       NamedCommands.registerCommand("speakerShot", null); // TODO fill in command !

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
