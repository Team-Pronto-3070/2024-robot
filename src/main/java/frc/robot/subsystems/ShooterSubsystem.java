package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterModule;

public class ShooterSubsystem extends SubsystemBase {

  double ampRPM = Constants.Shooter.Motor.ampSpeed;
  double speakerRPM = Constants.Shooter.Motor.speakerSpeed;
  double rightMod = 1.0;

  private final ShooterModule MotorLeft;

  private final ShooterModule MotorRight;

  public ShooterSubsystem() {
    MotorLeft = new ShooterModule(Constants.Shooter.leftMotorID);
    MotorRight = new ShooterModule(Constants.Shooter.rightMotorID);

    SmartDashboard.putNumber("Speaker Note RPM", speakerRPM);
    SmartDashboard.putNumber("Amp Note RPM", ampRPM);
    SmartDashboard.putNumber("Right Flywheel Multiplier", rightMod);
  }

  public void launchSpeakerNote() {
    speakerRPM =
      SmartDashboard.getNumber(
        "Speaker Note RPM",
        Constants.Shooter.Motor.speakerSpeed
      );
    rightMod = SmartDashboard.getNumber("Right Flywheel Multiplier", 1.0);

    MotorLeft.setRPM(speakerRPM);
    MotorRight.setRPM(speakerRPM * -rightMod);
  }

  public void launchAmpNote() {
    ampRPM =
      SmartDashboard.getNumber(
        "Amp Note RPM",
        Constants.Shooter.Motor.ampSpeed
      );

    MotorLeft.setRPM(ampRPM);
    MotorRight.setRPM(ampRPM * -1);
  }

  public void stop() {
    MotorLeft.stop();
    MotorRight.stop();
  }
}
