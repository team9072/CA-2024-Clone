package frc.robot.autonomous;

import java.util.function.Supplier;

import frc.robot.autonomous.modes.AutoModeBase;
import frc.robot.autonomous.modes.DoNothingMode;
import frc.robot.autonomous.modes.IntakeTestMode;
import frc.robot.autonomous.modes.ShootPreloadedMode;
import frc.robot.autonomous.modes.ShootThenDriveMode;
import frc.robot.autonomous.tasks.Task;

public class AutoRunner {
  private static AutoRunner m_autoRunner = null;
  private AutoModeBase m_autoMode;

  public static AutoRunner getInstance() {
    if (m_autoRunner == null) {
      m_autoRunner = new AutoRunner();
    }
    return m_autoRunner;
  }

  public enum AutoMode {
    DO_NOTHING("Do Nothing", () -> new DoNothingMode()),
    INTAKE_TEST("Test Intake", () -> new IntakeTestMode()),
    SHOOT_ONLY("Shoot Only", () -> new ShootPreloadedMode()),
    SHOOT_AMP_CORNER("Shoot & Leave (Amp Side)", () -> new ShootThenDriveMode("Amp Corner & Leave")),
    SHOOT_SOURCE_CORNER("Shoot & Leave (Source Side)", () -> new ShootThenDriveMode("Source Corner & Leave"));
    
    public final String modeName;
    public final Supplier<AutoModeBase> autoMode;

    AutoMode(String modeName, Supplier<AutoModeBase> modeSupplier) {
      this.modeName = modeName;
      this.autoMode = modeSupplier;
    }
  }

  public Task getNextTask() {
    return m_autoMode.getNextTask();
  }

  public void setAutoMode(AutoMode mode) {
    m_autoMode = mode.autoMode.get();

    m_autoMode.queueTasks();
    m_autoMode.setStartingPose();
  }
}
