package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoLongShotTrack extends SequentialCommandGroup {

    public ShooterAutoLongShotTrack(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new InstantCommand(()-> Limelight.getInstance().setPipeline(Constants.LIMELIGHT_LONG_PIPELINE)),
                new ParallelCommandGroup(
                        new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_LONG_RPM, Constants.SHOOTER_KICKER_LONG_RPM),
                        new TurretSetToTrackGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_LONG_SHOT_ANGLE_DEGREES),
                        new HoodSetAngle(shooter, Constants.HOOD_LONG_ANGLE_DEGREES)
                ),
                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_LONG_SHOT_DEGREES),
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new MagazineSetRPMRotations(magazine, Constants.MAGAZINE_SHOOT_AUTO_RPM,
                        magazineRotations)
        );
    }
}