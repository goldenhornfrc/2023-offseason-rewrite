package frc.lib.util;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;

public final class CTREConfigs {
  public CANCoderConfiguration swerveCANCoderConfig;

  public CTREConfigs() {
    swerveCANCoderConfig = new CANCoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCANCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
    swerveCANCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
