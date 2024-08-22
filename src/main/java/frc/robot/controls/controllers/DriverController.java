package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public class DriverController extends FilteredController {
  
  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Axis
  private final double k_triggerActivationThreshold = 0.5;

  public double getForwardAxis() {
    return -this.getFilteredAxis(Axis.kLeftY);
  }

  public double getLeftAxis() {
    return -this.getFilteredAxis(Axis.kLeftY);
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(Axis.kRightX);
  }

  public boolean getWantsFullIntake() {
    return this.getButton(Button.kA);
  }

  public boolean getWantsIntake() {
    return this.getButton(Button.kRightBumper);
  }

  public boolean getWantsEject() {
    return this.getButton(Button.kB);
  }

  public boolean getWantsSource() {
    return this.getButton(Button.kX);
  }

  public boolean getWantsStow() {
    return this.getButton(Button.kY);
  }

  public boolean getWantsMoreSpeed() {
    return this.getHatUp();
  }

  public boolean getWantsLessSpeed() {
    return this.getHatDown();
  }

  public boolean getWantsShooterStop() {
    return this.getButton(Button.kLeftBumper);
  }

  public boolean getWantsSpeedMode() {
    return this.getFilteredAxis(Axis.kLeftTrigger) > k_triggerActivationThreshold;
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(Axis.kRightTrigger) > k_triggerActivationThreshold;
  }
}
