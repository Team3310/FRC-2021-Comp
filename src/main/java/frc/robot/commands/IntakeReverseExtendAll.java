package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class IntakeReverseExtendAll extends SequentialCommandGroup {

    public IntakeReverseExtendAll(Intake intake, Magazine magazine) {

        addCommands(
                new TurretSetAngle(Turret.getInstance(), Constants.TURRET_INTAKE_ANGLE_DEGREES),
                new IntakeSetRPM(intake, Constants.INTAKE_REVERSE_RPM)
        );
    }
}