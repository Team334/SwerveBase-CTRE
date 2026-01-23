// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.Filesystem;
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
  public static final Frequency simNotifierFrequency = Hertz.of(200);

  public static class Ports {
    public static final int driverController = 0;
  }

  public static class FieldConstants {
    // public static final AprilTagFieldLayout tagLayout =
    //     AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // uncomment if using the test tag layout
    public static final AprilTagFieldLayout tagLayout;

    static {
      try {
        tagLayout =
            new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/test-tag-layout.json");
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
    }
  }

  public static class VisionConstants {
    public static final double singleTagStdDevsScaler = 3;

    public static final double ambiguityThreshold = 0.2;

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 5;
    public static final double zBoundMargin = 0.1;

    public static final String leftArducamName = "left-arducam";
    public static final String rightArducamName = "right-arducam";

    public static final VisionPoseEstimatorConstants leftArducam =
        new VisionPoseEstimatorConstants(
            leftArducamName,
            new Transform3d(
                new Translation3d(0.3015, 0.3014, 0.199),
                new Rotation3d(0, -Units.degreesToRadians(16.96), -Units.degreesToRadians(15))),
            0.1,
            2,
            4.5);

    public static final VisionPoseEstimatorConstants rightArducam =
        new VisionPoseEstimatorConstants(
            rightArducamName,
            new Transform3d(
                new Translation3d(0.3015, -0.3014, 0.199),
                new Rotation3d(0, -Units.degreesToRadians(16.96), Units.degreesToRadians(15))),
            0.1,
            2,
            4.5);
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Mass mass = Pounds.of(136.38);
    public static final MomentOfInertia moi = KilogramSquareMeters.of(8.777);

    public static final LinearVelocity driverTranslationalVelocity = MetersPerSecond.of(4);
    public static final AngularVelocity driverAngularVelocity = RadiansPerSecond.of(Math.PI);

    public static final LinearVelocity profileTranslationalVelocity = MetersPerSecond.of(3);
    public static final LinearAcceleration profileTranslationalAcceleration =
        MetersPerSecondPerSecond.of(6);

    public static final AngularVelocity profileAngularVelocity = RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration profileAngularAcceleration =
        RadiansPerSecondPerSecond.of(4 * Math.PI);

    public static final Per<LinearVelocityUnit, DistanceUnit> poseTranslationalkP =
        MetersPerSecond.per(Meter).ofNative(8);
    public static final Per<AngularVelocityUnit, AngleUnit> poseRotationkP =
        RadiansPerSecond.per(Radian).ofNative(8);

    public static final boolean ignorePoseTolerance = true;

    public static final Translation2d poseTranslationTolerance = new Translation2d(0.03, 0.03);
    public static final Rotation2d poseRotationTolerance = Rotation2d.fromDegrees(1);

    public static LinearVelocity translationalDeadband = MetersPerSecond.of(0.01);
    public static AngularVelocity rotationalDeadband = RadiansPerSecond.of(0.01);
  }
}
