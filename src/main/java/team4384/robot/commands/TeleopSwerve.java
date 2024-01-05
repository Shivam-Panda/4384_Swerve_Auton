package team4384.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import team4384.robot.constants.SwerveMap;
import team4384.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private JoystickButton SlowMode;
    private JoystickButton resetGyro;
    private JoystickButton NinetyPreset;
    private DoubleSupplier Inverted;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier Inverted, JoystickButton SlowMode, JoystickButton resetGyro, JoystickButton NinetyPreset) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.SlowMode = SlowMode;
        this.resetGyro = resetGyro;
        this.Inverted = Inverted;
        this.NinetyPreset = NinetyPreset;
    }
    @Override
    public void execute() {
        /* Get Values, Deadband*/
        if(NinetyPreset.getAsBoolean()) {
            s_Swerve.setModuleStates(SwerveMap.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0.5), new Translation2d(0, Rotation2d.fromDegrees(90)));
        }
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;

        if (SlowMode.getAsBoolean()) {
            translationVal = -MathUtil.applyDeadband(MathUtil.clamp(translationSup.getAsDouble(), -SwerveMap.maxSlowTeleSpeed, SwerveMap.maxSlowTeleSpeed), SwerveMap.translateDeadband);
            strafeVal = -MathUtil.applyDeadband(MathUtil.clamp(strafeSup.getAsDouble(), -SwerveMap.maxSlowTeleSpeed, SwerveMap.maxSlowTeleSpeed), SwerveMap.translateDeadband);

        }
        else  {
            translationVal = MathUtil.applyDeadband(MathUtil.clamp(translationSup.getAsDouble(), -SwerveMap.maxTeleSpeed, SwerveMap.maxTeleSpeed), SwerveMap.translateDeadband) * -1;
            strafeVal = MathUtil.applyDeadband(MathUtil.clamp(strafeSup.getAsDouble(), -SwerveMap.maxTeleSpeed, SwerveMap.maxTeleSpeed), SwerveMap.translateDeadband) * -1;
        }
        rotationVal = MathUtil.applyDeadband(MathUtil.clamp(rotationSup.getAsDouble(), -SwerveMap.maxRotSpeed , SwerveMap.maxRotSpeed), SwerveMap.rotateDeadband);

        double Invert = Inverted.getAsDouble();

        if (Invert == 1 ||Invert == -1) {
            translationVal *= Invert;
            strafeVal *= Invert;
        }

        if (resetGyro.getAsBoolean()) {
            s_Swerve.zeroGyro();
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveMap.maxSpeed), 
            rotationVal * SwerveMap.maxAngularVelocity,
            true,
            true
        );
    }
}