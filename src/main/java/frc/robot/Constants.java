// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Frequency simUpdateFrequency = Hertz.of(200);

  public static class Ports {
    public static final int driverController = 0;
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(100);

    public static final LinearVelocity translationalDeadband = MetersPerSecond.of(0.1);
    public static final AngularVelocity rotationalDeadband = RadiansPerSecond.of(Math.PI * 0.1);

    public static final LinearVelocity maxTranslationSpeed = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(1);
  }
}
