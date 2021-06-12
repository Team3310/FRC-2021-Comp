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
        public double magazineRPM = Constants.MAGAZINE_SHOOT_RPM;
    }

    public ShooterSetShot(Shooter shooter, Magazine magazine, Turret turret, ShooterParams params) {
        addCommands(
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new InstantCommand(()->Limelight.getInstance().setPipeline(params.limelightPipeline)),
                new ShooterSetReady(shooter,false),
                new ShooterSetRPM(shooter, params.shooterRPM, params.kickerRPM),
                new HoodSetAngle(shooter, params.hoodAngle),
                new InstantCommand(()->magazine.setMagazineRPMValue(params.magazineRPM)),
                new SequentialCommandGroup(
                    new TurretSetToGyroAngle(turret, params.turretGyroOffset),
                    new TurretSetToTrackLimelightAngle(turret, params.limelightOffset, params.isLimelightActive),
                    new ShooterSetReady(shooter, true)
                )
        );
    }
}