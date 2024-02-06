

package frc.robot;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {
    public static final class IntakeSubsystem {
        public static final double runningSpeed = 0.00;
        public static final int limitSwitchPort = 0;
        public static final double runningTime = 0;
        public static final int intakeMotorID = 0;
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final SparkLimitSwitch.Type limitSwitchPolarity = SparkLimitSwitch.Type.kNormallyClosed;

        public static final int driverPort = 0;
    }

    public static final class OI {
        public static final int driverPort = 0;

    }
}
