package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.commands.ResetOdometryAuto;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ResetAllHomePositions extends SequentialCommandGroup {

    public ResetAllHomePositions(Drive drive, Turret turret, Magazine magazine, Shooter shooter, Climb climb) {
        addCommands(
                new InstantCommand(()-> shooter.resetHoodHomePosition()),
                new ResetOdometryAuto(),
                new InstantCommand(()-> magazine.resetHomePosition()),
                new InstantCommand(()-> turret.resetHomePosition(Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES)),
                new InstantCommand(()-> drive.resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES))
        );
    }
}