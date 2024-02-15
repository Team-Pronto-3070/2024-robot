package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterModule;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterModule MotorLeft;

  private final ShooterModule MotorRight;

  public ShooterSubsystem() {
    MotorLeft = new ShooterModule(Constants.Shooter.leftMotorID);
    MotorRight = new ShooterModule(Constants.Shooter.rightMotorID);
  }

  public void launchSpeakerNote() {
    MotorLeft.setRPM(Constants.Shooter.Motor.speakerSpeed);
    MotorRight.setRPM(Constants.Shooter.Motor.speakerSpeed * -1);
  }

  public void launchAmpNote() {
    MotorLeft.setRPM(Constants.Shooter.Motor.ampSpeed);
    MotorRight.setRPM(Constants.Shooter.Motor.ampSpeed * -1);
  }
}
