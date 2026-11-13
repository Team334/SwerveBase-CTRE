// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.fault;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.fault.FaultsTable.Fault;
import frc.lib.fault.FaultsTable.FaultType;
import frc.lib.util.CTREUtil;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/**
 * FaultLogger allows for faults and errors to be logged and displayed.
 *
 * <pre>
 * FaultLogger.register(talonfx); // registers a TalonFX, periodically checking for hardware faults.
 * </pre>
 */
public final class FaultLogger {
  @FunctionalInterface
  public static interface FaultReporter {
    void report();
  }

  private static boolean _enableConsole = true;

  // DATA
  private static final List<FaultReporter> faultReporters = new ArrayList<>();
  // signals grouped by CAN bus name, since refreshAll cannot span buses
  private static final Map<String, List<BaseStatusSignal>> faultSignals = new HashMap<>();
  private static final Set<Fault> newFaults = new HashSet<>();
  private static final Set<Fault> activeFaults = new HashSet<>();
  private static final Set<Fault> totalFaults = new HashSet<>();

  // NETWORK TABLES
  private static FaultsTable activeAlerts;
  private static FaultsTable totalAlerts;

  private static boolean _hasBeenSetup = false;

  /** Must be called to setup the fault logger. */
  public static void setup(NetworkTableInstance ntInst) {
    if (_hasBeenSetup) return;

    var base = ntInst.getTable("FaultLogger");

    activeAlerts = new FaultsTable(base, "Active Faults");
    totalAlerts = new FaultsTable(base, "Total Faults");

    _hasBeenSetup = true;
  }

  /** Must be called to setup the fault logger. */
  public static void setup() {
    setup(NetworkTableInstance.getDefault());
  }

  /** Whether or not to print new faults into the console. */
  public static void enableConsole(boolean enable) {
    _enableConsole = enable;
  }

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    DogLog.time("Timing/FaultLogger/update()");

    // reportError=false: a non-OK status is reported once as a fault instead of a DS print every
    // poll
    for (var entry : faultSignals.entrySet()) {
      var status = BaseStatusSignal.refreshAll(false, entry.getValue());
      if (!status.isOK()) {
        report(
            "CANBus ("
                + entry.getKey()
                + ")- Fault Signals Not Received: "
                + status.getDescription(),
            FaultType.ERROR);
      }
    }
    faultReporters.forEach(f -> f.report());

    if (!newFaults.equals(activeFaults)) {
      for (Fault f : newFaults) {
        if (activeFaults.contains(f)) continue;

        if (_enableConsole) {
          switch (f.type()) {
            case ERROR -> DriverStation.reportError("[Fault Logger] " + f.toString(), false);
            case WARNING -> DriverStation.reportWarning("[Fault Logger] " + f.toString(), false);
            case INFO -> System.out.println("[Fault Logger] " + f.toString());
          }
        }

        DogLog.logFault(f.toString(), null);
      }

      for (Fault f : activeFaults) {
        if (!newFaults.contains(f)) DogLog.decreaseFault(f.toString());
      }

      activeFaults.clear();
      activeFaults.addAll(newFaults);
      totalFaults.addAll(activeFaults);

      // don't log to NT if there is a match going on (just use doglog)
      if (!DriverStation.isFMSAttached()) {
        activeAlerts.set(activeFaults);
        totalAlerts.set(totalFaults);
      }
    }

    newFaults.clear();

    DogLog.timeEnd("Timing/FaultLogger/update()");
  }

  /** Clears total faults. */
  public static void clear() {
    activeFaults.forEach(f -> DogLog.decreaseFault(f.toString()));
    totalFaults.clear();
    activeFaults.clear();
    newFaults.clear();

    // update() only publishes on change, so publish the emptied sets here
    if (!DriverStation.isFMSAttached() && activeAlerts != null) {
      activeAlerts.set(activeFaults);
      totalAlerts.set(totalFaults);
    }
  }

  /** Clears fault suppliers. */
  public static void unregisterAll() {
    faultReporters.clear();
    faultSignals.clear();
  }

  /**
   * Returns the set of all current faults.
   *
   * @return The set of all current faults.
   */
  public static Set<Fault> activeFaults() {
    return activeFaults;
  }

  /**
   * Returns the set of all total faults.
   *
   * @return The set of all total faults.
   */
  public static Set<Fault> totalFaults() {
    return totalFaults;
  }

  /**
   * Reports a fault. It surfaces (console/DogLog/NT) at the next {@link #update()} poll.
   * FaultLogger must be used from the main robot thread only.
   *
   * @param fault The fault to report.
   */
  public static void report(Fault fault) {
    newFaults.add(fault);
  }

  /**
   * Reports a fault.
   *
   * @param description The description of the fault.
   * @param type The type of the fault.
   */
  public static void report(String description, FaultType type) {
    report(new Fault(description, type));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultReporters.add(() -> supplier.get().ifPresent(FaultLogger::report));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   */
  public static void register(BooleanSupplier condition, String description, FaultType type) {
    faultReporters.add(
        () -> {
          if (condition.getAsBoolean()) {
            report(description, type);
          }
        });
  }

  private static void register(
      ParentDevice device, StatusSignal<Boolean> signal, String description, FaultType type) {
    faultSignals.computeIfAbsent(device.getNetwork().getName(), k -> new ArrayList<>()).add(signal);
    register(signal::getValue, description, type);
  }

  /**
   * Registers a new TalonFX.
   *
   * @param talonFX The TalonFX.
   */
  public static void register(TalonFX talonFX) {
    String name = CTREUtil.getName(talonFX);

    var hardware = talonFX.getFault_Hardware(false);
    var boot = talonFX.getFault_BootDuringEnable(false);
    var deviceTemp = talonFX.getFault_DeviceTemp(false);
    var procTemp = talonFX.getFault_ProcTemp(false);
    var undervoltage = talonFX.getFault_Undervoltage(false);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                4, hardware, boot, deviceTemp, procTemp, undervoltage),
        talonFX);

    register(talonFX, hardware, name + "- Hardware Fault.", FaultType.ERROR);
    register(talonFX, boot, name + "- Boot While Enabling.", FaultType.WARNING);
    register(talonFX, deviceTemp, name + "- Device Temperature Too High.", FaultType.WARNING);
    register(talonFX, procTemp, name + "- Processor Temp Too High.", FaultType.WARNING);
    register(
        talonFX, undervoltage, name + "- Voltage Too Low, Check For Brownouts.", FaultType.WARNING);
  }

  /**
   * Registers a new CANcoder.
   *
   * @param cancoder The CANcoder.
   */
  public static void register(CANcoder cancoder) {
    String name = CTREUtil.getName(cancoder);

    var hardware = cancoder.getFault_Hardware(false);
    var badMagnet = cancoder.getFault_BadMagnet(false);
    var boot = cancoder.getFault_BootDuringEnable(false);
    var undervoltage = cancoder.getFault_Undervoltage(false);

    CTREUtil.attempt(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(4, hardware, badMagnet, boot, undervoltage),
        cancoder);

    register(cancoder, hardware, name + "- Hardware Fault.", FaultType.ERROR);
    register(cancoder, badMagnet, name + "- Bad Magnet Signal.", FaultType.ERROR);
    register(cancoder, boot, name + "- Boot While Enabling.", FaultType.WARNING);
    register(
        cancoder,
        undervoltage,
        name + "- Voltage Too Low, Check For Brownouts.",
        FaultType.WARNING);
  }

  /**
   * Registers a new Pigeon.
   *
   * @param pigeon The Pigeon.
   */
  public static void register(Pigeon2 pigeon) {
    String name = CTREUtil.getName(pigeon);

    var hardware = pigeon.getFault_Hardware(false);
    var boot = pigeon.getFault_BootDuringEnable(false);
    var undervoltage = pigeon.getFault_Undervoltage(false);

    CTREUtil.attempt(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(4, hardware, boot, undervoltage), pigeon);

    register(pigeon, hardware, name + "- Hardware Fault.", FaultType.ERROR);
    register(pigeon, boot, name + "- Boot While Enabling.", FaultType.WARNING);
    register(
        pigeon, undervoltage, name + "- Voltage Too Low, Check For Brownouts.", FaultType.WARNING);
  }

  /**
   * Registers a new PhotonCamera (more detailed logs on the web ui).
   *
   * @param photonCamera The PhotonCamera.
   */
  public static void register(PhotonCamera photonCamera) {
    register(
        () -> !photonCamera.isConnected(),
        "PhotonCamera " + photonCamera.getName() + "- Disconnected.",
        FaultType.ERROR);
  }

  /**
   * Registers a new CAN bus, logging its utilization and error counts.
   *
   * @param bus The CAN bus.
   */
  public static void register(CANBus bus) {
    String name = bus.getName();

    faultReporters.add(
        () -> {
          // blocks up to 1 ms, fine at the 1 s poll
          var status = bus.getStatus();

          DogLog.log("CANBus/" + name + "/Utilization", status.BusUtilization);
          DogLog.log("CANBus/" + name + "/Bus Off Count", status.BusOffCount);
          DogLog.log("CANBus/" + name + "/Tx Full Count", status.TxFullCount);
          DogLog.log("CANBus/" + name + "/REC", status.REC);
          DogLog.log("CANBus/" + name + "/TEC", status.TEC);

          if (!status.Status.isOK()) {
            report("CANBus (" + name + ")- " + status.Status.getDescription(), FaultType.ERROR);
          }
        });
  }
}
