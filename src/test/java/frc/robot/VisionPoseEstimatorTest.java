package frc.robot;

import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.setupTests;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.UnitTestingUtil;
import frc.robot.utils.VisionPoseEstimator;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionPoseEstimatorTest {
  private VisionPoseEstimator _testCam;
  private VisionSystemSim _visionSystemSim;

  private static AprilTagFieldLayout _fieldLayout;

  // dummy gyro heading function for disambiguation
  private Rotation2d dummyGyroHeading(double t) {
    return Rotation2d.kZero;
  }

  @BeforeAll
  public static void setupField() {
    List<AprilTag> tags = new ArrayList<>();

    // // add all tags to the field layout
    tags.add(new AprilTag(1, new Pose3d(1, 0, 1, new Rotation3d(0, 0, -Math.PI))));

    _fieldLayout = new AprilTagFieldLayout(tags, Units.feetToMeters(54), Units.feetToMeters(27));
  }

  @BeforeEach
  public void setup() {
    setupTests();

    var testCam =
        new VisionPoseEstimatorConstants(
            "test-cam", new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()), 0.2, 0.0001);

    _testCam =
        VisionPoseEstimator.buildFromConstants(testCam, UnitTestingUtil.getNtInst(), _fieldLayout);

    _visionSystemSim = new VisionSystemSim("");
    _visionSystemSim.addCamera(_testCam.getCameraSim(), _testCam.robotToCam);
  }

  @AfterEach
  public void close() throws Exception {
    reset(_testCam);
  }

  @Test
  public void noResults() {
    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update(this::dummyGyroHeading);

    // no targets, no new estimates
    assertEquals(_testCam.getNewEstimates().size(), 0);
  }

  @Test
  public void singleTagResult() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero); // should see the single target

    _testCam.update(this::dummyGyroHeading);

    assertEquals(_testCam.getNewEstimates().size(), 1);
  }

  @Test
  public void singleTagResults() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero);
    _visionSystemSim.update(Pose2d.kZero); // should see two new results of the single target

    _testCam.update(this::dummyGyroHeading);

    assertEquals(_testCam.getNewEstimates().size(), 2);
  }

  @Test
  public void singleTagEstimate() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update(this::dummyGyroHeading);

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(estimates.size(), 1);

    var estimate = estimates.get(0);

    assert estimate.isValid();

    // pose validity
    assertEquals(estimate.pose().getX(), 0, 1e-5);
    assertEquals(estimate.pose().getY(), 0, 1e-5);
    assertEquals(estimate.pose().getZ(), 0, 1e-5);
    assertEquals(estimate.pose().getRotation().getX(), 0, 1e-5);
    assertEquals(estimate.pose().getRotation().getY(), 0, 1e-5);
    assertEquals(estimate.pose().getRotation().getZ(), 0, 1e-5);

    // should see only ID 1
    assertArrayEquals(estimate.detectedTags(), new int[] {1});

    // distance validity
    assertEquals(
        estimate.avgTagDistance(),
        _fieldLayout
            .getTagPose(1)
            .get()
            .getTranslation()
            .getDistance(Pose3d.kZero.getTranslation()),
        1e-5);

    // std devs validity
    assertNotEquals(estimate.stdDevs(), new int[] {0, 0, 0});
  }

  @Test
  public void disambiguation() {}

  @Test
  public void estimateSort() {}

  // and so on ... TODO
}
