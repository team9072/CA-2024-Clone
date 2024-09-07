package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private static final double k_pivotMotorP = 0;
  private static final double k_pivotMotorI = 0;
  private static final double k_pivotMotorD = 0;

  // Volts
  private static final double k_pivotMotorKs = 0;
  private static final double k_pivotMotorKg = 0.29;
  // Volts per (rads per second)
  private static final double k_pivotMotorKv = 0;//1.58;
  // Volts per (rads per second squared)
  private static final double k_pivotMotorKa = 0;

  // Rads per second
  private static final double k_pivotMotorMaxSpeed = 0.25;
  // Rads per second squared
  private static final double k_pivotMotorMaxAccel = 0.1;

  private final TrapezoidProfile m_pivotProfile = new TrapezoidProfile(new Constraints(k_pivotMotorMaxSpeed, k_pivotMotorMaxAccel));
  private final ArmFeedforward m_pivotFeedforward = new ArmFeedforward(k_pivotMotorKs, k_pivotMotorKg, k_pivotMotorKv, k_pivotMotorKa);
  private final SparkPIDController m_pivotPid;
  private final SparkAbsoluteEncoder m_pivotEncoder;


  private final DigitalInput m_IntakeLimitSwitch = new DigitalInput(Constants.Intake.k_intakeLimitSwitchId);

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
    mPivotMotor.setSmartCurrentLimit(10);

    m_pivotEncoder = mPivotMotor.getAbsoluteEncoder();
    // Rads and rads per sec
    m_pivotEncoder.setPositionConversionFactor(Math.PI * 2);
    m_pivotEncoder.setVelocityConversionFactor((Math.PI * 2) / 60.0);
    m_pivotEncoder.setZeroOffset(m_pivotEncoder.getZeroOffset() + Constants.Intake.k_pivotEncoderOffset);
    m_pivotEncoder.setInverted(false);

    m_pivotPid = mPivotMotor.getPIDController();
    m_pivotPid.setFeedbackDevice(m_pivotEncoder);
    m_pivotPid.setP(k_pivotMotorP);
    m_pivotPid.setI(k_pivotMotorI);
    m_pivotPid.setD(k_pivotMotorD);
    m_pivotPid.setOutputRange(-0.1, 0.1, 0);

    m_periodicIO = new PeriodicIO();
    SmartDashboard.putNumber("pivot goal", 0);
  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    State intake_pivot_setpoint = new State();
    boolean intake_pivot_enabled = false;
    double intake_pivot_feedforward = 0.0;
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
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

    // Pivot control
    double pivot_goal = pivotTargetToAngle(m_periodicIO.pivot_target);
    pivot_goal = SmartDashboard.getNumber("pivot goal", 0);
    State setpoint = m_pivotProfile.calculate(0.05, m_periodicIO.intake_pivot_setpoint, new State(pivot_goal, 0));
    m_periodicIO.intake_pivot_feedforward = m_pivotFeedforward.calculate(setpoint.position, setpoint.velocity);
    m_periodicIO.intake_pivot_setpoint = setpoint;
    m_periodicIO.intake_pivot_enabled = true;// m_periodicIO.pivot_target != PivotTarget.NONE;

    // Intake control
    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
  }

  @Override
  public void writePeriodicOutputs() {
    if (m_periodicIO.intake_pivot_enabled) {
      m_pivotPid.setReference(
        m_periodicIO.intake_pivot_setpoint.position,
        ControlType.kPosition,
        0,
        m_periodicIO.intake_pivot_feedforward
      );
    } else {
      mPivotMotor.disable();
    }

    mIntakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void stop() {
    // TODO: reset setpoint to current measure on change target
    m_periodicIO.pivot_target = PivotTarget.NONE;
    m_periodicIO.intake_pivot_enabled = false;
    m_periodicIO.intake_speed = 0.0;
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed", intakeStateToSpeed(m_periodicIO.intake_state));
    putString("State", m_periodicIO.intake_state.toString());
    putNumber("Pivot/Abs Position", getPivotAngleRadians());
    putNumber("Pivot/Setpoint", m_periodicIO.intake_pivot_setpoint.position);
    putNumber("Pivot/Goal", pivotTargetToAngle(m_periodicIO.pivot_target));
    putNumber("Pivot/Feedforward", m_periodicIO.intake_pivot_feedforward);
    putNumber("Pivot/Current", mPivotMotor.getOutputCurrent());

    putBoolean("Limit Switch", getIntakeHasNote());
  }

  @Override
  public void reset() {
  }

  public double pivotTargetToAngle(PivotTarget target) {
    //FIXME: Change constants
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
      case SOURCE:
        return Constants.Intake.k_pivotAngleSource;
      case AMP:
        return Constants.Intake.k_pivotAngleAmp;
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
    return m_pivotEncoder.getPosition() + Constants.Intake.k_pivotEncoderOffset;
  }

  public double getPivotSpeedRadians() {
    return m_pivotEncoder.getVelocity();
  }

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    //TODO: Note detection
    return false;
    //return !m_IntakeLimitSwitch.get();
  }

  // Pivot helper functions
  public void goToGround() {
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_periodicIO.intake_state = IntakeState.INTAKE;
    m_leds.setColor(Color.kYellow);
  }

  public void goToSource() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToAmp() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToStow() {
    m_periodicIO.pivot_target = PivotTarget.STOW;
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

  public void setPivotTarget(PivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && isPivotAtTarget()) {
      m_periodicIO.pivot_target = PivotTarget.STOW;
      m_periodicIO.intake_state = IntakeState.NONE;
      m_leds.setColor(Color.kGreen);
    }
  }

  private boolean isPivotAtTarget() {
    return Math.abs(getPivotAngleRadians() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
  }
}
