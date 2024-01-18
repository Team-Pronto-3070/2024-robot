package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Velocity;

public class Constants {
    public static final class ClimberSubsystem {
        public static final int leftTalonID = 0; // TODO
        public static final int rightTalonID = 0; // TODO

        public static final TalonSRXConfiguration leftTalonConfig  = new TalonSRXConfiguration();
        public static final TalonSRXConfiguration rightTalonConfig = new TalonSRXConfiguration();

        // TODO fill in
        public static final double maxSpeed = 0.2; // in percent output
    }
}
