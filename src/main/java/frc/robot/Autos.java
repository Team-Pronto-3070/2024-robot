package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
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
    }
}
