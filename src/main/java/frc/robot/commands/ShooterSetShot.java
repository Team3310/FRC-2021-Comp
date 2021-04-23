package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterSetShot extends ParallelCommandGroup {

    public static class ShooterParams{
        public double hoodAngle;
        public double shooterRPM;
        public double kickerRPM;
        public double limelightOffset;
        public int limelightPipeline;
        public double turretGyroOffset;
        public boolean isLimelightActive = true;
    }

    public ShooterSetShot(Shooter shooter, Magazine magazine, Turret turret, ShooterParams params) {
        addCommands(
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new InstantCommand(()->Limelight.getInstance().setPipeline(params.limelightPipeline)),
                new ShooterSetReady(shooter,false),
                new ShooterSetRPMDistance(shooter),
                new MagazineSetRPM(magazine, Constants.MAGAZINE_SHOOT_RPM),
                //               new SequentialCommandGroup(
 //                       new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES)
 //                       new MagazineIndexDividerToTurretTrack(magazine)
 //               ),
 //               new ShooterSetCachedHoodAngle(shooter, Constants.HOOD_MEDIUM_ANGLE_DEGREES),
 //               new TurretSetCachedLimelightOffset(turret, Constants.LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES),
                new HoodSetAngleDistance(shooter),
                new SequentialCommandGroup(
                        new TurretSetToGyroAngle(turret, params.turretGyroOffset),
                        new TurretSetToTrackLimelightAngle(turret, params.limelightOffset, params.turretGyroOffset, params.isLimelightActive),
                        new ShooterSetReady(shooter, true)
                )
        );
    }
}