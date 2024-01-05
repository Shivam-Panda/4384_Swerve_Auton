package team4384.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4384.robot.RobotContainer;
import team4384.robot.constants.SwerveMap;

import java.util.List;

public class Autonomous {
    private Swerve s_Swerve;
    private AHRS gyro;
    public Autonomous(Swerve m_swerve, AHRS gyro) {
        this.s_Swerve = m_swerve;
        this.gyro = gyro;
    }
    private static double microTime(double seconds) {
        return seconds * 1000000;
    }

    private void follower() {

    }

    public void basic() {
    }
}
