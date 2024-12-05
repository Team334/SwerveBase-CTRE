package frc.robot;

import static frc.lib.UnitTestingUtil.reset;

import frc.robot.Constants.VisionConstants;
import frc.robot.utils.VisionPoseEstimator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class VisionPoseEstimatorTest {
  private VisionPoseEstimator _testCam;

  @BeforeEach
  public void setup() {
    _testCam = VisionPoseEstimator.buildFromConstants(VisionConstants.testCam);
  }

  @AfterEach
  public void close() throws Exception {
    reset(_testCam);
  }

  @Test
  public void noTargets() {
    // TODO
  }

  @Test
  public void disambiguation() {
    // TODO
  }

  @Test
  public void estimateSort() {
    // TODO
  }

  // and so on ...
}
