package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.XboxController.Button;

public class OperatorController extends FilteredController {

  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Buttons
  public boolean getWantsMoreSpeed() {
    return this.getButtonPressed(Button.kRightBumper);
  }

  public boolean getWantsLessSpeed() {
    return false;
  }

  public boolean getWantsShooterStop() {
    return this.getButtonPressed(Button.kLeftBumper);
  }

  // D pad

  public boolean getWantsClimberRelease() {
    return this.getHatDown();
  }

  public boolean getWantsClimberTiltRight() {
    return this.getHatRight();
  }

  public boolean getWantsClimberClimb() {
    return this.getHatUp();
  }

  public boolean getWantsClimberTiltLeft() {
    return this.getHatLeft();
  }
}
