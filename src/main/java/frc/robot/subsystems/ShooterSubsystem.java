package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterModule;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterModule MotorLeft;

  private final ShooterModule MotorRight;

  public ShooterSubsystem() {
    MotorLeft = new ShooterModule(Constants.Shooter.Left.motorID);
    MotorRight = new ShooterModule(Constants.Shooter.Right.motorID);
  }

  public void launchNote() {
    MotorLeft.setRPM(Constants.Shooter.Motor.speed);
    MotorRight.setRPM(Constants.Shooter.Motor.speed * -1);
  }
}
