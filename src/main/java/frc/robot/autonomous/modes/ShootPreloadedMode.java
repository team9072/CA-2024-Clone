package frc.robot.autonomous.modes;

public class ShootPreloadedMode extends AutoModeBase {

    @Override
    public void queueTasks() {
        queueTask(shooterSequence());
    }
}
