package frc.robot;

import static frc.lib.UnitTestingUtil.reset;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.VisionPoseEstimator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.simulation.VisionSystemSim;

public class VisionPoseEstimatorTest {
  private VisionPoseEstimator _testCam;
  private VisionSystemSim _visionSystemSim;

  // dummy gyro heading function for disambiguation
  private Rotation2d dummyGyroHeading(double t) {
    return Rotation2d.kZero;
  }

  @BeforeEach
  public void setup() {
    _testCam = VisionPoseEstimator.buildFromConstants(VisionConstants.testCam);

    _visionSystemSim = new VisionSystemSim("");
    _visionSystemSim.addCamera(_testCam.getCameraSim(), _testCam.robotToCam);
  }

  @AfterEach
  public void close() throws Exception {
    reset(_testCam, _visionSystemSim.getDebugField());
  }

  @Test
  public void noResults() {
    _visionSystemSim.update(Pose2d.kZero); // no targets here

    _testCam.update(this::dummyGyroHeading);

    assertEquals(_testCam.getNewEstimates().size(), 0);
  }

  // TODO

  @Test
  public void singleTagResult() {}

  @Test
  public void singleTagResults() {}

  @Test
  public void disambiguation() {}

  @Test
  public void estimateSort() {}

  // and so on ...
}
