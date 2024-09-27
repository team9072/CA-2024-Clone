package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class ShootThenDriveMode extends ShootPreloadedMode {
    private final String m_pathName;

    public ShootThenDriveMode(String pathName) {
        m_pathName = pathName;
    }
    
    @Override
    public void queueTasks() {
        //TODO: Uncomment queueShooterTask();
        queueTask(new DriveTrajectoryTask(m_pathName));
    }
}
