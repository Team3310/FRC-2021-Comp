package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;


public class MagazineSetRPMToValue extends CommandBase {
    private final Magazine magazine;

    public MagazineSetRPMToValue(Magazine magazine) {
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        magazine.setMagazineRPM(magazine.getMagazineValue());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
