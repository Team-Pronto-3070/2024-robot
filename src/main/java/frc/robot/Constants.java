package frc.robot;
    
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;


public class Constants {
    public static final class ClimberSubsystem {
        public static final int leftTalonID = 0; // TODO
        public static final int rightTalonID = 0; // TODO

        public static final TalonSRXConfiguration leftTalonConfig  = new TalonSRXConfiguration();
        public static final TalonSRXConfiguration rightTalonConfig = new TalonSRXConfiguration();

        // TODO fill in
        public static final double climbSpeed = 0.2; // in percent output
        public static final double maxSpeed = 0.4; // in percent output

        public static final double balanceAdjustQuotient = 0.5; // TODO

    }
}
