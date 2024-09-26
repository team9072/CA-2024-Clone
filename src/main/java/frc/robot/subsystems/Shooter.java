package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private CANSparkFlex mLeftShooterMotor;
  private CANSparkFlex mRightShooterMotor;

  private RelativeEncoder mLeftShooterEncoder;
  private RelativeEncoder mRightShooterEncoder;

  private Shooter() {
    super("Shooter");

    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new CANSparkFlex(Constants.kShooterLeftMotorId, MotorType.kBrushless);
    mRightShooterMotor = new CANSparkFlex(Constants.kShooterRightMotorId, MotorType.kBrushless);
    mLeftShooterMotor.restoreFactoryDefaults();
    mRightShooterMotor.restoreFactoryDefaults();

    mLeftShooterEncoder = mLeftShooterMotor.getEncoder();
    mRightShooterEncoder = mRightShooterMotor.getEncoder();

    mLeftShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    mRightShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    mLeftShooterMotor.setInverted(true);
    mRightShooterMotor.setInverted(false);

  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    double limitedSpeed = mPeriodicIO.shooter_rpm / 10000;
    mLeftShooterMotor.set(limitedSpeed);
    mRightShooterMotor.set(limitedSpeed);
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    putNumber("Left speed:", mLeftShooterEncoder.getVelocity());
    putNumber("Right speed:", mRightShooterEncoder.getVelocity());
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    mPeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
