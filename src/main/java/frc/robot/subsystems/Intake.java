package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.leds.LEDs;

public class Intake extends Subsystem {
  private static final double k_pivotMotorP = 4;
  private static final double k_pivotMotorI = 0;
  private static final double k_pivotMotorD = 0;

  // Volts
  private static final double k_pivotMotorKs = 0.132;
  private static final double k_pivotMotorKg = -0.5;
  // Volts per (rads per second)
  private static final double k_pivotMotorKv = 1.8;

  // Rads per second
  private static final double k_pivotMotorMaxSpeed = 16;
  private static final double k_pivotMotorStowMaxSpeed = 2;
  // Rads per second squared
  private static final double k_pivotMotorMaxAccel = 10;
  private static final double k_pivotMotorStowMaxAccel = 2;

  private final Constraints k_deployConstraints = new Constraints(k_pivotMotorMaxSpeed, k_pivotMotorMaxAccel);
  private final Constraints k_stowConstraints = new Constraints(k_pivotMotorStowMaxSpeed, k_pivotMotorStowMaxAccel);

  private final ProfiledPIDController m_pivotController = new ProfiledPIDController(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD, k_deployConstraints);


  private final ArmFeedforward m_pivotFeedforward = new ArmFeedforward(k_pivotMotorKs, k_pivotMotorKg, k_pivotMotorKv);
  private final SparkAbsoluteEncoder m_pivotEncoder;


  private final DigitalInput m_NoteDetectionBeamBreak = new DigitalInput(Constants.Intake.k_beamBreakDioId);

  public final LEDs m_leds = LEDs.getInstance();

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO m_periodicIO;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private CANSparkMax mIntakeMotor;
  private CANSparkMax mPivotMotor;

  private Intake() {
    super("Intake");

    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setInverted(true);
    mPivotMotor.setSmartCurrentLimit(10);
    // Make encoder data frequency match rio loop frequency
    mPivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    m_pivotEncoder = mPivotMotor.getAbsoluteEncoder();
    // Rads and rads per sec
    m_pivotEncoder.setPositionConversionFactor(Math.PI * 2);
    m_pivotEncoder.setVelocityConversionFactor((Math.PI * 2) / 60.0);
    m_pivotEncoder.setInverted(false);

    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.NONE;
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    State intake_pivot_goal = new State();
    State intake_pivot_setpoint = new State();
    boolean intake_pivot_enabled = false;
    double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();
    pivotPeriodic();

    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
  }

  private void pivotPeriodic() {
    double pivot_goal = pivotTargetToAngle(m_periodicIO.pivot_target);
    m_periodicIO.intake_pivot_goal.position = pivot_goal;
    double currentAngle = getPivotAngleRadians();

    Constraints constraints = currentAngle <= Constants.Intake.k_pivotAngleSlow ? k_stowConstraints : k_deployConstraints;
    double feedbackVoltage = m_pivotController.calculate(currentAngle, m_periodicIO.intake_pivot_goal, constraints);
    feedbackVoltage += Math.signum(feedbackVoltage) * k_pivotMotorKs;
    State setpoint = m_pivotController.getSetpoint();
    double feedforwardVoltage = m_pivotFeedforward.calculate(currentAngle, setpoint.velocity);
    
    m_periodicIO.intake_pivot_voltage = feedforwardVoltage + feedbackVoltage;
    m_periodicIO.intake_pivot_setpoint = setpoint;
    m_periodicIO.intake_pivot_enabled =  m_periodicIO.pivot_target != PivotTarget.NONE;

  }

  @Override
  public void writePeriodicOutputs() {
    if (m_periodicIO.intake_pivot_enabled) {
      mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);
    } else {
      mPivotMotor.disable();
    }

    mIntakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void stop() {
    setPivotTarget(PivotTarget.NONE);
    stopIntake();
    m_periodicIO.intake_pivot_enabled = false;
    m_periodicIO.intake_speed = 0.0;
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed", intakeStateToSpeed(m_periodicIO.intake_state));
    putString("State", m_periodicIO.intake_state.toString());
    putNumber("Pivot/Abs Position", getPivotAngleRadians());
    putNumber("Pivot/Encoder Velocity", getPivotSpeedRadians());
    putNumber("Pivot/Goal", m_periodicIO.intake_pivot_goal.position);
    putNumber("Pivot/Setpoint", m_periodicIO.intake_pivot_setpoint.position);
    putNumber("Pivot/Set Velocity", m_periodicIO.intake_pivot_setpoint.velocity);
    putNumber("Pivot/PID Error", m_pivotController.getPositionError());
    putNumber("Pivot/Output Voltage", m_periodicIO.intake_pivot_voltage);
    putNumber("Pivot/Current", mPivotMotor.getOutputCurrent());

    putBoolean("Beam Break", getIntakeHasNote());
  }

  @Override
  public void reset() {

  }

  /**
   * Resets the pivot's motion profile.
   */
  public void resetPivotMotion() {
    m_periodicIO.intake_pivot_setpoint = new State(getPivotAngleRadians(), getPivotSpeedRadians());
    m_pivotController.reset(m_periodicIO.intake_pivot_setpoint);
  }

  public void setPivotTarget(PivotTarget target) {
    m_periodicIO.pivot_target = target;
    resetPivotMotion();
  }

  private double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
      case STOW:
        return Constants.Intake.k_pivotAngleStow;
      default:
        // "Safe" default
        return Units.degreesToRadians(0);
    }
  }

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          return Constants.Intake.k_intakeSpeed;
        }
        return 0.0;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }

  public double getPivotAngleRadians() {
    double recordedPosition = m_pivotEncoder.getPosition() + Constants.Intake.k_pivotEncoderOffset;
    // Change the modulus from -pi to pi to -pi-1 to pi-1
    // This fixes the issue of the encoder wrapping when at the zero point
    return MathUtil.angleModulus(recordedPosition + 1) - 1;
  }

  public double getPivotSpeedRadians() {
    return m_pivotEncoder.getVelocity();
  }

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    return !m_NoteDetectionBeamBreak.get();
  }

  // Pivot helper functions
  public void goToGround() {
    setPivotTarget(PivotTarget.GROUND);
    m_periodicIO.intake_state = IntakeState.INTAKE;
    m_leds.setColor(Color.kYellow);
  }

  public void goToStow() {
    setPivotTarget(PivotTarget.STOW);
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void 
  eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_speed = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    SmartDashboard.putNumber("Current", mIntakeMotor.getOutputCurrent());
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && atTarget()) {
      setPivotTarget(PivotTarget.STOW);
      m_periodicIO.intake_state = IntakeState.NONE;
      m_leds.setColor(Color.kGreen);
    }
  }

  private boolean atTarget() {
    return m_pivotController.atGoal();
  }

  private boolean atSetpoint() {
    return m_pivotController.atSetpoint();
  }
}
