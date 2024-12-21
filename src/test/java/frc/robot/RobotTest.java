package frc.robot;

import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.FileLogger;
import edu.wpi.first.epilogue.logging.MultiLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotTest {
  private Robot _robot;

  @BeforeEach
  public void setup() {
    setupTests();

    _robot = new Robot(getNtInst());
  }

  @AfterEach
  public void close() throws Exception {
    reset(_robot);
  }

  @Test
  public void fmsFileOnly() {
    // at the start should be both nt and file logging
    assert Epilogue.getConfig().dataLogger instanceof MultiLogger; // multilogger setup
    assert DogLog.getOptions().ntPublish();

    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();

    assert DriverStation.isFMSAttached();

    _robot.robotPeriodic();

    // once the fms connects, it should be file only
    assert Epilogue.getConfig().dataLogger instanceof FileLogger;
    assertFalse(DogLog.getOptions().ntPublish());
  }
}
