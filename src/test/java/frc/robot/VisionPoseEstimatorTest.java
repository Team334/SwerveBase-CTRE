package frc.robot;

import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.setupTests;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.UnitTestingUtil;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.VisionPoseEstimator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionPoseEstimatorTest {
  private VisionPoseEstimator _testCam;
  private VisionSystemSim _visionSystemSim;

  // dummy gyro heading function for disambiguation
  private Rotation2d dummyGyroHeading(double t) {
    return Rotation2d.kZero;
  }

  @BeforeEach
  public void setup() {
    setupTests();

    _testCam =
        VisionPoseEstimator.buildFromConstants(
            VisionConstants.testCam, UnitTestingUtil.getNtInst());

    _visionSystemSim = new VisionSystemSim("");
    _visionSystemSim.addCamera(_testCam.getCameraSim(), _testCam.robotToCam);
  }

  @AfterEach
  public void close() throws Exception {
    reset(_testCam);
  }

  @Test
  public void noResults() {
    _visionSystemSim.update(Pose2d.kZero); // no targets here

    _testCam.update(this::dummyGyroHeading);

    assertEquals(_testCam.getNewEstimates().size(), 0);
  }

  @Test
  public void singleTagResult() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(
            new Pose3d(1, 0, 1, new Rotation3d(0, 0, -Math.PI)), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero); // should see the single target

    _testCam.update(this::dummyGyroHeading);

    assertEquals(_testCam.getNewEstimates().size(), 1);
  }

  // TODO

  @Test
  public void singleTagResults() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(
            new Pose3d(1, 0, 1, new Rotation3d(0, 0, -Math.PI)), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero);
    _visionSystemSim.update(Pose2d.kZero); // should see two new results of a single target

    _testCam.update(this::dummyGyroHeading);

    assertEquals(_testCam.getNewEstimates().size(), 2);
  }

  @Test
  public void disambiguation() {}

  @Test
  public void estimateSort() {}

  // and so on ...
}
