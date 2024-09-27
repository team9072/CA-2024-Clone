package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class ShootThenDriveMode extends AutoModeBase {
    private final String m_pathName;

    public ShootThenDriveMode(String pathName) {
        m_pathName = pathName;
    }
    
    @Override
    public void queueTasks() {
        queueTask(shooterSequence());
        queueTask(new DriveTrajectoryTask(m_pathName));
    }
}
