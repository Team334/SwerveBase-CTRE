// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;

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

  public static class FieldConstants {
    public static final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  public static class VisionConstants {
    public static final String leftArducamName = "left-arducam";

    public static final double[] singleTagBaseStdDevs = new double[] {5, 5, 5};
    public static final double[] multiTagBaseStdDevs = new double[] {1, 1, 1};

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.1;

    public static final VisionPoseEstimatorConstants leftArducam =
        new VisionPoseEstimatorConstants(
            leftArducamName,
            new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()),
            0.2,
            0.0001);
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Distance driveRadius =
        Meters.of(
            Math.sqrt(
                Math.pow(TunerConstants.FrontLeft.LocationX, 2)
                    + Math.pow(TunerConstants.FrontLeft.LocationY, 2)));

    public static final LinearVelocity maxTranslationalSpeed = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(Math.PI);

    public static final LinearVelocity translationalDeadband = maxTranslationalSpeed.times(0.1);
    public static final AngularVelocity rotationalDeadband = maxAngularSpeed.times(0.1);
  }
}
