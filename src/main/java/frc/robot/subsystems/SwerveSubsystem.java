package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.util.ADIS16470_IMU;
import frc.robot.util.SwerveDriveKinematics2;
import frc.robot.util.SwerveModule;
import frc.robot.util.SwerveModuleState2;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

//import org.photonvision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearLeft;
  private final SwerveModule rearRight;

  private final ADIS16470_IMU gyro;
  private double gyroOffset = 0.0;

  public final SwerveDriveKinematics2 kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field;

  public SwerveSubsystem() {
    frontLeft =
      new SwerveModule(
        Constants.Swerve.FrontLeft.driveID,
        Constants.Swerve.FrontLeft.turnID,
        Constants.Swerve.FrontLeft.offset
      );

    frontRight =
      new SwerveModule(
        Constants.Swerve.FrontRight.driveID,
        Constants.Swerve.FrontRight.turnID,
        Constants.Swerve.FrontRight.offset
      );

    rearLeft =
      new SwerveModule(
        Constants.Swerve.RearLeft.driveID,
        Constants.Swerve.RearLeft.turnID,
        Constants.Swerve.RearLeft.offset
      );

    rearRight =
      new SwerveModule(
        Constants.Swerve.RearRight.driveID,
        Constants.Swerve.RearRight.turnID,
        Constants.Swerve.RearRight.offset
      );

    gyro = new ADIS16470_IMU();

    kinematics =
      new SwerveDriveKinematics2(
        new Translation2d(
          Constants.Swerve.wheelBase / 2,
          Constants.Swerve.trackWidth / 2
        ),
        new Translation2d(
          Constants.Swerve.wheelBase / 2,
          -Constants.Swerve.trackWidth / 2
        ),
        new Translation2d(
          -Constants.Swerve.wheelBase / 2,
          Constants.Swerve.trackWidth / 2
        ),
        new Translation2d(
          -Constants.Swerve.wheelBase / 2,
          -Constants.Swerve.trackWidth / 2
        )
      );

    poseEstimator =
      new SwerveDrivePoseEstimator(
        kinematics,
        getYaw(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition(),
        },
        new Pose2d()
      );

    field = new Field2d();
    SmartDashboard.putData("field", field);


    // * autos:

    //  
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getCurrentSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(
          Constants.Autos.translation_P,
          Constants.Autos.translation_I,
          Constants.Autos.translation_D),
        new PIDConstants(
          Constants.Autos.rotation_P,
          Constants.Autos.rotation_I,
          Constants.Autos.rotation_D),
        Constants.Autos.maxSpeed,
        Constants.Autos.driveBaseRadius,
        new ReplanningConfig()),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getAngle() - gyroOffset);
  }

  public double getPitch() {
    //return gyro.getYComplementaryAngle();
    //return gyro.getAngle(gyro.getPitchAxis());
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
      frontLeft.getState(),
      frontRight.getState(),
      rearLeft.getState(),
      rearRight.getState()
    );
  }

  public void resetOdometry(Pose2d pose) {
    gyro.setGyroAngle(gyro.getYawAxis(), pose.getRotation().getDegrees());
    poseEstimator.resetPosition(
      getYaw(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition(),
      },
      pose
    );
    //gyroOffset = gyro.getAngle() - pose.getRotation().getDegrees();
  }

  public void stop() {
    drive(0, 0, 0, false, false);
  }

  public void drive(
    double xSpeed,
    double ySpeed,
    double rot,
    boolean fieldRelative,
    boolean isOpenLoop
  ) {
    SmartDashboard.putNumber("chassis omega", rot);
    SmartDashboard.putNumber("chassis target mps", xSpeed);
    var swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SmartDashboard.putNumber(
      "front left pre desaturate target mps",
      swerveModuleStates[0].speedMetersPerSecond
    );
    SwerveDriveKinematics2.desaturateWheelSpeeds(
      swerveModuleStates,
      Constants.Swerve.maxSpeed
    );

    SmartDashboard.putNumber(
      "front left target mps",
      swerveModuleStates[0].speedMetersPerSecond
    );

    frontLeft.setDesiredState(swerveModuleStates[0], isOpenLoop);
    frontRight.setDesiredState(swerveModuleStates[1], isOpenLoop);
    rearLeft.setDesiredState(swerveModuleStates[2], isOpenLoop);
    rearRight.setDesiredState(swerveModuleStates[3], isOpenLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public void setModuleStates(SwerveModuleState2[] desiredStates) {
    SwerveDriveKinematics2.desaturateWheelSpeeds(
      desiredStates,
      Constants.Swerve.maxSpeed
    );
    frontLeft.setDesiredState(desiredStates[0], false);
    frontRight.setDesiredState(desiredStates[1], false);
    rearLeft.setDesiredState(desiredStates[2], false);
    rearRight.setDesiredState(desiredStates[3], false);
  }

  /*
    public void addPotentialVisionMeasurement(Optional<EstimatedRobotPose> potentialVisionEstimate) {
        if (potentialVisionEstimate.isPresent()) {
            EstimatedRobotPose camPose = potentialVisionEstimate.get();
            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            field.getObject("vision estimate").setPose(camPose.estimatedPose.toPose2d());
        } else {
            field.getObject("vision estimate").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }
    }
    */

  @Override
  public void periodic() {
    poseEstimator.update(
      getYaw(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition(),
      }
    );

    field.setRobotPose(getPose());

    SmartDashboard.putNumber(
      "front left speed",
      frontLeft.getState().speedMetersPerSecond
    );
    SmartDashboard.putNumber(
      "front right speed",
      frontRight.getState().speedMetersPerSecond
    );
    SmartDashboard.putNumber(
      "rear left speed",
      rearLeft.getState().speedMetersPerSecond
    );
    SmartDashboard.putNumber(
      "rear right speed",
      rearRight.getState().speedMetersPerSecond
    );
    SmartDashboard.putNumber(
      "front left angle",
      frontLeft.getState().angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "front right angle",
      frontRight.getState().angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "rear left angle",
      rearLeft.getState().angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "rear right angle",
      rearRight.getState().angle.getDegrees()
    );

    SmartDashboard.putNumber("pitch", getPitch());
    SmartDashboard.putNumber("yaw", getYaw().getDegrees());
    SmartDashboard.putNumber("mod yaw", getYaw().getDegrees() % 360.0);
    SmartDashboard.putNumber("manual yaw", gyro.getAngle());
    SmartDashboard.putNumber("manual pitch", gyro.getAngle());
    SmartDashboard.putNumber("manual roll", gyro.getAngle());
  }
}

  private Pose2d getPose() {
    // TODO
    return null;
  }

  private void resetPose(Pose2d pose) {
    // TODO
  }

  private ChassisSpeeds getCurrentSpeeds() {
    // TODO
    return null;
  }

  private void driveRobotRelative(ChassisSpeeds chassisspeeds) {
    // TODO
  }

}
