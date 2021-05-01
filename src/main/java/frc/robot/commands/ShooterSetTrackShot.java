package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.utilities.Util;

public class ShooterSetTrackShot extends SequentialCommandGroup {

    public static class ShooterParams{
        public double hoodAngle;
        public double shooterRPM;
        public double kickerRPM;
        public double limelightOffset;
        public int limelightPipeline;
        public double turretGyroOffset;
        public boolean isLimelightActive = true;
    }

    public ShooterSetTrackShot(Shooter shooter, Magazine magazine, Turret turret, ShooterParams params, Drive drive) {
        addCommands(
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new InstantCommand(()->Limelight.getInstance().setPipeline(params.limelightPipeline)),
                new ShooterSetReady(shooter,false),
 //               new MagazineSetRPM(magazine, Constants.MAGAZINE_SHOOT_RPM),
                //               new SequentialCommandGroup(
 //                       new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES)
 //                       new MagazineIndexDividerToTurretTrack(magazine)
 //               ),
 //               new ShooterSetCachedHoodAngle(shooter, Constants.HOOD_MEDIUM_ANGLE_DEGREES),
 //               new TurretSetCachedLimelightOffset(turret, Constants.LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES),
                new TurretSetToGoalAngle(turret, drive),
                new TurretSetToTrackLimelightAngle(turret, 0, 0, true),
                new ShooterSetEverything(shooter),
                new ShooterSetReady(shooter, true)
        );
    }
}