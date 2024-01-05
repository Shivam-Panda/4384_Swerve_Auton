package team4384.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team4384.robot.RobotContainer;
import team4384.robot.constants.AutoConstants;
import team4384.robot.constants.SwerveMap;

import java.util.List;

public class Autonomous extends CommandBase {
    private Swerve s_Swerve;
    private ProfiledPIDController rotPID;
    private HolonomicDriveController hController;
    private PathPlannerTrajectory.PathPlannerState state;
    private Pose2d currentPose;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private PathPlannerTrajectory path;

    private Timer timer = new Timer();

    public Autonomous(Swerve m_swerve, String pathName) {
        this.s_Swerve = m_swerve;
        path = PathPlanner.loadPath(pathName, SwerveMap.maxSpeed, SwerveMap.maxAcceleration);
        rotPID = AutoConstants.rot_controller;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        hController = new HolonomicDriveController(AutoConstants.X_controller, AutoConstants.Y_controller, rotPID);

        timer.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        var currTime = timer.get();
        state = (PathPlannerTrajectory.PathPlannerState) path.sample(currTime);
        currentPose = s_Swerve.getPose();
        speeds = hController.calculate(currentPose, state, state.holonomicRotation);
        s_Swerve.setModuleStates(SwerveMap.swerveKinematics.toSwerveModuleStates(speeds));
    }
}
