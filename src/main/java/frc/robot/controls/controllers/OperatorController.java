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
  public boolean getWantsSpinShooter() {
    return this.getButtonPressed(Button.kRightBumper);
  }

  public boolean getWantsLessSpeed() {
    return this.getButtonPressed(Button.kBack);
  }

  public boolean getWantsIntakeDown() {
    return this.getButton(Button.kA);
  }

  public boolean getWantsIntakeEject() {
    return this.getButton(Button.kB);
  }

  public boolean getWantsIntakeStow() {
    return this.getButton(Button.kX);
  }

  public boolean getWantsIntakeAtAmp() {
    return this.getButton(Button.kY);
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
