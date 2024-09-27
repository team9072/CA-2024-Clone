package frc.robot.autonomous.modes;

public class IntakeTestMode extends AutoModeBase {

    @Override
    public void queueTasks() {
        queueTask(intakeDown());
    }
    
}
