package org.firstinspires.ftc.teamcode.commands.turret;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretAutoOdoCMD extends TurretToPosCMD {
    public TurretAutoOdoCMD(TurretSubsystem subsystem) {
        super(subsystem, 0d, false);
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        targetPos = (subsystem.getTurretToGoalAngle())+3.5;
        super.execute();
    }
}
