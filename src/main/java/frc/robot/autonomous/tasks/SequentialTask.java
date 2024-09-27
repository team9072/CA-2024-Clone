package frc.robot.autonomous.tasks;

public class SequentialTask extends Task {
    private final Task[] k_tasks;
    private Task m_currentTask = null;
    private int m_index;

    public SequentialTask(Task... tasks) {
        this.k_tasks = tasks;
    }

    @Override
    public void start() {
        m_index = 0;
        m_currentTask = k_tasks[m_index];
    }

    @Override
    public void update() {
        m_currentTask.update();
        if (m_currentTask.isFinished()) {
            m_currentTask.done();
            m_index++;
            m_currentTask = m_index < k_tasks.length ? k_tasks[m_index] : null;
        }
    }

    @Override
    public void updateSim() {
        m_currentTask.updateSim();
    }

    @Override
    public boolean isFinished() {
        return m_index >= k_tasks.length;
    }
    
}
