package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;

public class OI {
    private CommandXboxController driver;
    public final Trigger buttonStatus;

    public OI(int driverPort) {
        driver = new CommandXboxController(driverPort);
        buttonStatus = driver.x();
    }
}
