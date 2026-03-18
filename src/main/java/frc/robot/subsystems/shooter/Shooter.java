package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
<<<<<<< HEAD
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.MathUtil;
=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple shooter subsystem with 4 motors:
 * - Two Krakens (implemented here with SparkFlex) with CAN IDs 5 and 6
 * - Two NEOs (Spark Max) with CAN IDs 7 and 9
 *
 * Pressing operator A will spin all shooter wheels (wiring is done in RobotContainer).
 */
public class Shooter extends SubsystemBase {
  // IDs: Krakens use TalonFX controllers with built-in encoders
  private static final int KRAKEN_1_ID = 5;
  private static final int KRAKEN_2_ID = 6;
  private static final int NEO_1_ID = 7;
  private static final int NEO_2_ID = 9; // moved to 9 to avoid conflict with hopper

<<<<<<< HEAD
  // Starting speed used by the operator preset as a normalized fraction ([-1,1]).
  // Default to 0.6 (60%) so the Krakens spin up at a safe, repeatable fraction
  // when the operator presses the A preset.
  private static final double DEFAULT_SPEED = 0.6;
  // Per-motor tuning multipliers: keep these at 1.0 by default so the
  // drive-ratio math produces explicit, predictable outputs. Previously
  // large scale factors caused normalization to cancel expected behavior.
  private static final double KRAKEN_SPEED_SCALE = 1.0;
  private static final double NEO_SPEED_SCALE = 1.0;
=======
  // Starting speed used by the operator preset as a high-but-safe initial command.
  // Set slightly below 1.0 so the shooter can spin up reliably without immediately
  // demanding an absolute maximum.
  private static final double DEFAULT_SPEED = 1.50;
  // Per-motor tuning multipliers: bias group outputs to better match wheel
  // linear speeds given the differing wheel diameters. These starting values
  // are recommended for initial on-robot testing; fine-tune empirically.
  private static final double KRAKEN_SPEED_SCALE = 1.3; // increase Kraken contribution
  private static final double NEO_SPEED_SCALE = 1.6; // NEO baseline
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
  // Counter-rotation direction multipliers for Krakens only. NEO directions
  // are handled via per-controller inversion (configured below) for clarity.
  // These signs define the current "shoot" direction for the rear Kraken wheels.
  private static final double KRAKEN_1_DIR = 1.0;
  private static final double KRAKEN_2_DIR = -1.0;
  // Pulley ratios (motor:wheel). Wheel speed = motor speed * (motorPulley / wheelPulley).
  // motor:wheel is the ratio motorPulley/wheelPulley. To compute the motor
  // output required to achieve a wheel speed fraction `s`, we use
  // motorOutput = s * (1.0 / driveRatio) (then normalize if needed so outputs stay in [-1,1]).
  private static final double KRAKEN_DRIVE_RATIO = 24.0 / 30.0;
  private static final double NEO_DRIVE_RATIO = 1.0 / 2.0;

  private final SparkMax neo1 = new SparkMax(NEO_1_ID, MotorType.kBrushless);
  private final SparkMax neo2 = new SparkMax(NEO_2_ID, MotorType.kBrushless);

  // Krakens: use TalonFX (Phoenix 6) controllers which provide encoder reads
  private final TalonFX kraken1 = new TalonFX(KRAKEN_1_ID);
  private final TalonFX kraken2 = new TalonFX(KRAKEN_2_ID);

  public Shooter() {
    // Note: Kraken controllers are not configured here because they are not
    // represented by SparkFlex/SparkMax classes. If your Kraken vendor API
    // requires configuration, perform it here (or add a Kraken-specific
    // config wrapper). For the NEOs we still configure via SparkMaxConfig.
    // Configure NEO controllers. Use per-controller inversion so hardware
    // wiring/tach polarity is clear in config rather than sign hacks in code.
    SparkMaxConfig nConfig1 = new SparkMaxConfig();
<<<<<<< HEAD
  nConfig1.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    nConfig1.smartCurrentLimit(40);
  // Flip front NEO direction: swap inversion so neo spin directions are switched
  nConfig1.inverted(true); // was false
    SparkMaxConfig nConfig2 = new SparkMaxConfig();
  nConfig2.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    nConfig2.smartCurrentLimit(40);
  nConfig2.inverted(false); // was true
    neo1.configure(nConfig1, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
    neo2.configure(nConfig2, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
    // Configure TalonFX Slot0 PIDF/Feedforward for native velocity control.
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    krakenConfig.Slot0.kP = 0.06;
    krakenConfig.Slot0.kI = 0.0;
    krakenConfig.Slot0.kD = 0.0;
    krakenConfig.Slot0.kV = 1.0 / (MAX_KRAKEN_RPM / 60.0); // feedforward per RPS
    krakenConfig.Slot0.kS = 0.02;
    kraken1.getConfigurator().apply(krakenConfig);
    kraken2.getConfigurator().apply(krakenConfig);
  }

  // Kraken max physical speed (RPM). Use this as the requested max.
  private static final double MAX_KRAKEN_RPM = 6300.0;
  // NOTE: controller-native closed-loop is preferred for Krakens (TalonFX).
  // The TalonFX runs a 1 kHz control loop on the device; we configure Slot0
  // PIDF in the constructor and drive the controllers with VelocityVoltage.
  // Software PID state and constants were removed in favor of native control.
  private double requestedKrakenRPM = 0.0; // signed RPM request
  private boolean closedLoopEnabled = false;
  private boolean presetAInvertKraken = false;
  // Phoenix6 native-velocity control request objects (re-used to avoid allocations)
  private final VelocityVoltage kraken1VelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage kraken2VelocityRequest = new VelocityVoltage(0).withSlot(0);
  // Last requested wheel speed fraction ([-1,1]) from spinAll/spinAllPresetA.
  // Persisted so periodic() can re-apply outputs if the closed-loop toggle
  // changes mid-match and no new spinAll() call occurs.
  private double lastRequestedSpeed = 0.0;
  @Override
  public void periodic() {
    // Publish shooter diagnostics so operator A preset can be verified on the Dashboard.
    // Avoid wrapping the whole method in a broad try/catch as that can hide
    // connection or telemetry errors (which makes debugging on the driver
    // station much harder). Instead, protect only the risky reads and still
    // publish as many keys as possible.

    // Outputs
    try {
      SmartDashboard.putNumber("Shooter/Neo1Output", neo1.get());
    } catch (Exception e) {
      SmartDashboard.putString("Shooter/DiagError", "Neo1 output read failed: " + e.getMessage());
    }
    try {
      SmartDashboard.putNumber("Shooter/Neo2Output", neo2.get());
    } catch (Exception e) {
      SmartDashboard.putString("Shooter/DiagError", "Neo2 output read failed: " + e.getMessage());
    }

    // Kraken velocities (may throw if TalonFX not present); publish NaN on failure
    double k1 = Double.NaN;
    double k2 = Double.NaN;
    try {
      k1 = getKraken1Velocity();
    } catch (Exception e) {
      SmartDashboard.putString("Shooter/DiagError", "Kraken1Vel read failed: " + e.getMessage());
    }
    try {
      k2 = getKraken2Velocity();
    } catch (Exception e) {
      SmartDashboard.putString("Shooter/DiagError", "Kraken2Vel read failed: " + e.getMessage());
    }
    SmartDashboard.putNumber("Shooter/Kraken1Vel", k1);
    SmartDashboard.putNumber("Shooter/Kraken2Vel", k2);

    // NEO encoder velocities: try real RPM, otherwise fall back to output value.
    try {
      SmartDashboard.putNumber("Shooter/Neo1RPM", neo1.getEncoder().getVelocity());
    } catch (Exception e) {
      SmartDashboard.putNumber("Shooter/Neo1RPM", neo1.get());
    }
    try {
      SmartDashboard.putNumber("Shooter/Neo2RPM", neo2.getEncoder().getVelocity());
    } catch (Exception e) {
      SmartDashboard.putNumber("Shooter/Neo2RPM", neo2.get());
    }

    // Publish the last requested Kraken RPM and the cap
    double maxAllowedKrakenRPM = Preferences.getDouble("Shooter/MaxKrakenRPM", 5600.0);
    SmartDashboard.putNumber("Shooter/RequestedKrakenRPM", requestedKrakenRPM);
    SmartDashboard.putNumber("Shooter/MaxAllowedKrakenRPM", maxAllowedKrakenRPM);

    // Any active
    boolean anyActive = (Math.abs(neo1.get()) > 0.01) || (Math.abs(neo2.get()) > 0.01) || (!Double.isNaN(k1) && Math.abs(k1) > 1.0) || (!Double.isNaN(k2) && Math.abs(k2) > 1.0);
    SmartDashboard.putBoolean("Shooter/AnyActive", anyActive);

    // Closed-loop toggle & telemetry (Preference can be changed at runtime)
    // Enable TalonFX native closed-loop control by default; teams can
    // override at runtime via the Preferences entry "Shooter/UseClosedLoop".
    closedLoopEnabled = Preferences.getBoolean("Shooter/UseClosedLoop", true);
    SmartDashboard.putBoolean("Shooter/UseClosedLoop", closedLoopEnabled);
    SmartDashboard.putNumber("Shooter/RequestedKrakenRPM_Current", requestedKrakenRPM);

    // Ensure Krakens are commanded every periodic cycle based on the last
    // requested speed and the current closed-loop toggle.
    refreshKrakenFromLastRequest();
  }

  /** Apply Kraken outputs based on the last requested speed and current mode. */
  private void refreshKrakenFromLastRequest() {
    double s = lastRequestedSpeed;
    // If closed-loop enabled, compute requested RPM and drive TalonFX velocity.
    if (closedLoopEnabled) {
      double requestedRPM = s * MAX_KRAKEN_RPM;
      double maxAllowed = Preferences.getDouble("Shooter/MaxKrakenRPM", 5600.0);
      requestedRPM = Math.copySign(Math.min(Math.abs(requestedRPM), maxAllowed), requestedRPM);
      requestedKrakenRPM = requestedRPM;
      runKrakenClosedLoop();
      return;
    }

    // Open-loop: compute Kraken duty from the last requested speed and apply.
    double krakenRaw = s / KRAKEN_DRIVE_RATIO; // motor fraction required for Kraken motors
    double neoRaw = s / NEO_DRIVE_RATIO; // used to compute normalization consistently with spinAll
    krakenRaw *= KRAKEN_SPEED_SCALE;
    neoRaw *= NEO_SPEED_SCALE;
    double maxDemand = Math.max(Math.abs(krakenRaw), Math.abs(neoRaw));
    double scale = (maxDemand > 1.0) ? (1.0 / maxDemand) : 1.0;
    double krakenOut = krakenRaw * scale;
    // Apply any configured direction multipliers per controller and preset inversion.
    double krakenSet = MathUtil.clamp(krakenOut, -1.0, 1.0);
    int invert = presetAInvertKraken ? -1 : 1;
    kraken1.setControl(new DutyCycleOut(krakenSet * KRAKEN_1_DIR * invert));
    kraken2.setControl(new DutyCycleOut(krakenSet * KRAKEN_2_DIR * invert));
  }

  private void runKrakenClosedLoop() {
    // Use TalonFX native velocity control. The TalonFX expects velocity in
    // rotations per second (RPS) for this wrapper; convert from requested RPM.
    double targetRPS = requestedKrakenRPM / 60.0;
    int invert = presetAInvertKraken ? -1 : 1;
    // Drive the TalonFX with the pre-allocated VelocityVoltage requests.
    kraken1.setControl(kraken1VelocityRequest.withVelocity(targetRPS * KRAKEN_1_DIR * invert));
    kraken2.setControl(kraken2VelocityRequest.withVelocity(targetRPS * KRAKEN_2_DIR * invert));
  }

  /** Read NEO encoder velocity (RPM) if available via the Spark wrapper. */
  public double getNeo1Velocity() {
    try {
      return neo1.getEncoder().getVelocity();
    } catch (Exception e) {
      return neo1.get();
    }
  }

  /** Read NEO encoder velocity (RPM) if available via the Spark wrapper. */
  public double getNeo2Velocity() {
    try {
      return neo2.getEncoder().getVelocity();
    } catch (Exception e) {
      return neo2.get();
=======
    nConfig1.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    nConfig1.smartCurrentLimit(40);
  nConfig1.inverted(false); // Front NEO direction flipped for updated wheel layout
    SparkMaxConfig nConfig2 = new SparkMaxConfig();
    nConfig2.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    nConfig2.smartCurrentLimit(40);
  nConfig2.inverted(true); // Front NEO direction flipped for updated wheel layout
    neo1.configure(nConfig1, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
    neo2.configure(nConfig2, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Publish shooter diagnostics so operator A preset can be verified on the Dashboard.
    try {
      SmartDashboard.putNumber("Shooter/Neo1Output", neo1.get());
      SmartDashboard.putNumber("Shooter/Neo2Output", neo2.get());
      SmartDashboard.putNumber("Shooter/Kraken1Vel", getKraken1Velocity());
      SmartDashboard.putNumber("Shooter/Kraken2Vel", getKraken2Velocity());
      SmartDashboard.putBoolean("Shooter/AnyActive", Math.abs(neo1.get()) > 0.01 || Math.abs(neo2.get()) > 0.01 || Math.abs(getKraken1Velocity()) > 1.0 || Math.abs(getKraken2Velocity()) > 1.0);
    } catch (Exception e) {
      // Ignore telemetry errors to avoid noisy logs on hardware without telemetry support
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
    }
  }

  // Encoder-follower control intentionally removed for shooter safety.

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken1Position() {
    return kraken1.getRotorPosition().getValueAsDouble();
  }

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken2Position() {
    return kraken2.getRotorPosition().getValueAsDouble();
  }

  /** Read integrated Kraken encoder velocity (rotations per second, RPS). */
  public double getKraken1Velocity() {
    return kraken1.getRotorVelocity().getValueAsDouble();
  }

  /** Read integrated Kraken encoder velocity (rotations per second, RPS). */
  public double getKraken2Velocity() {
    return kraken2.getRotorVelocity().getValueAsDouble();
  }

  /** Spin all shooter wheels at default speed. */
  public void spinAll() {
    spinAll(DEFAULT_SPEED);
  }

  /** Spin all shooter wheels at the given speed ([-1,1]). */
  public void spinAll(double speed) {
    double s = Math.max(-1.0, Math.min(1.0, speed));
<<<<<<< HEAD
    // Remember the last requested speed so periodic() can re-apply outputs
    // if the closed-loop toggle changes without another spinAll() call.
    lastRequestedSpeed = s;
    // Ensure any preset-A inversion is cleared when running the regular spinAll path
    presetAInvertKraken = false;
=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
    // Convert requested wheel-speed fraction to motor outputs using pulley ratios.
    // The drive ratio is defined as motorPulley/wheelPulley so
    // wheelSpeed = motorSpeed * driveRatio. Therefore motorSpeed = wheelSpeed / driveRatio.
    // We'll compute the raw motor demands and then normalize them if any exceed 1.0.
  double krakenRaw = s / KRAKEN_DRIVE_RATIO; // motor fraction required for Kraken motors
  double neoRaw = s / NEO_DRIVE_RATIO; // motor fraction required for NEO motors

  // Apply per-group tuning to bias Krakens vs NEOs.
  krakenRaw *= KRAKEN_SPEED_SCALE;
  neoRaw *= NEO_SPEED_SCALE;

  // Find max absolute demand to determine if normalization is required.
  double maxDemand = Math.max(Math.abs(krakenRaw), Math.abs(neoRaw));
  double scale = (maxDemand > 1.0) ? (1.0 / maxDemand) : 1.0;

  double krakenOut = krakenRaw * scale;
  double neoOut = neoRaw * scale;

<<<<<<< HEAD
    // If closed-loop is enabled, set the requested RPM and let the PID loop drive the Krakens.
    double requestedRPM = s * MAX_KRAKEN_RPM;
    if (closedLoopEnabled) {
      // Clamp the requested RPM to the safety cap (Preference-driven)
      double maxAllowed = Preferences.getDouble("Shooter/MaxKrakenRPM", 5600.0);
      requestedRPM = Math.copySign(Math.min(Math.abs(requestedRPM), maxAllowed), requestedRPM);
      // Store the requested RPM (signed). NEOs are still driven open-loop here.
      requestedKrakenRPM = requestedRPM;
  // Use the computed NEO output rather than forcing full duty (±1.0).
  // This makes intent clearer: NEOs remain open-loop here but obey the
  // computed drive-ratio and normalization. Prefer switching these to
  // vendor-native closed-loop (SparkMax PID) later.
  double neoSet = MathUtil.clamp(neoOut, -1.0, 1.0);
  neo1.set(neoSet);
  neo2.set(neoSet);
      return;
    }
    // Closed-loop is not enabled: use the computed outputs rather than forcing full duty.
    double krakenSet = MathUtil.clamp(krakenOut, -1.0, 1.0);
    // Apply any configured direction multipliers per controller.
    kraken1.setControl(new DutyCycleOut(krakenSet * KRAKEN_1_DIR));
    kraken2.setControl(new DutyCycleOut(krakenSet * KRAKEN_2_DIR));
    double neoSet = MathUtil.clamp(neoOut, -1.0, 1.0);
    neo1.set(neoSet);
    neo2.set(neoSet);
  }

  /**
   * Preset-A shooter spin: same as spinAll but reverse the Kraken directions
   * relative to the default configuration. This flips only the Krakens so
   * you can change their counter-rotation for the operator "A" preset
   * without affecting other code paths that call spinAll.
   */
  public void spinAllPresetA() {
    spinAllPresetA(DEFAULT_SPEED);
  }

  public void spinAllPresetA(double speed) {
    double s = Math.max(-1.0, Math.min(1.0, speed));
    // Persist this preset's requested speed for periodic refresh.
    lastRequestedSpeed = s;
    double krakenRaw = s / KRAKEN_DRIVE_RATIO; // motor fraction required for Kraken motors
    double neoRaw = s / NEO_DRIVE_RATIO; // motor fraction required for NEO motors

    // Apply per-group tuning to bias Krakens vs NEOs.
    krakenRaw *= KRAKEN_SPEED_SCALE;
    neoRaw *= NEO_SPEED_SCALE;

    // Find max absolute demand to determine if normalization is required.
    double maxDemand = Math.max(Math.abs(krakenRaw), Math.abs(neoRaw));
    double scale = (maxDemand > 1.0) ? (1.0 / maxDemand) : 1.0;

    double krakenOut = krakenRaw * scale;
    double neoOut = neoRaw * scale;

    // For Preset A we invert the Kraken directions compared to defaults.
    double requestedRPM = s * MAX_KRAKEN_RPM;
    if (closedLoopEnabled) {
      // Clamp preset requested RPM to the safety cap
      double maxAllowed = Preferences.getDouble("Shooter/MaxKrakenRPM", 5600.0);
      requestedRPM = Math.copySign(Math.min(Math.abs(requestedRPM), maxAllowed), requestedRPM);
      // Store requested RPM and indicate Kraken inversion for Preset A.
      requestedKrakenRPM = requestedRPM;
      presetAInvertKraken = true;
  // Use computed NEO output in Preset A open-loop path as well.
  double neoSet = MathUtil.clamp(neoOut, -1.0, 1.0);
  neo1.set(neoSet);
  neo2.set(neoSet);
      return;
    }

    // Legacy open-loop: use computed outputs but invert Kraken directions for Preset A.
    double krakenSet = MathUtil.clamp(krakenOut, -1.0, 1.0);
    krakenSet = -krakenSet; // invert for preset A
    kraken1.setControl(new DutyCycleOut(krakenSet * KRAKEN_1_DIR));
    kraken2.setControl(new DutyCycleOut(krakenSet * KRAKEN_2_DIR));
    double neoSet = MathUtil.clamp(neoOut, -1.0, 1.0);
    neo1.set(neoSet);
    neo2.set(neoSet);
=======
    // Apply outputs. Krakens use explicit direction multipliers; NEO inversion is handled
    // in the controller config above (neo2 configured inverted=true).
    kraken1.setControl(new DutyCycleOut(krakenOut * KRAKEN_1_DIR));
    kraken2.setControl(new DutyCycleOut(krakenOut * KRAKEN_2_DIR));
    neo1.set(neoOut);
    neo2.set(neoOut);
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
  }

  /** Stop all shooter motors. */
  public void stop() {
    // Stop TalonFX outputs
<<<<<<< HEAD
    // Clear requested RPM and PID state when stopping.
  requestedKrakenRPM = 0.0;
  // Clear last requested speed so periodic() doesn't immediately re-command motors
  // after a stop() call. lastRequestedSpeed is a normalized fraction in [-1,1].
  lastRequestedSpeed = 0.0;
  presetAInvertKraken = false;
=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
    kraken1.setControl(new DutyCycleOut(0.0));
    kraken2.setControl(new DutyCycleOut(0.0));
    neo1.stopMotor();
    neo2.stopMotor();
  }
}
