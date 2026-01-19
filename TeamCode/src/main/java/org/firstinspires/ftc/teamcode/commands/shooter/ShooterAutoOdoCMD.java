package org.firstinspires.ftc.teamcode.commands.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ShooterAutoOdoCMD extends CommandBase {

    private static InterpLUT VelocityLUT = new InterpLUT();

    static {
        VelocityLUT.add(0, 0);

        VelocityLUT.add(36, 1100);
        VelocityLUT.add(54, 1120);
        VelocityLUT.add(88, 1230);
        VelocityLUT.add(122, 1410);
        VelocityLUT.add(160, 1560); // new
        VelocityLUT.add(205, 1640);

        VelocityLUT.add(1000000000, 10000);


        VelocityLUT.createLUT();
    }

    double distance;
    private final ShooterSubsystem subsystem;

    private final TurretSubsystem turretSubsystem;

    public ShooterAutoOdoCMD(ShooterSubsystem subsystem, TurretSubsystem turretSubsystem) {
        this.subsystem = subsystem;
        this.turretSubsystem = turretSubsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute(){

        distance = turretSubsystem.getDistanceToGoal();

        double velocity = VelocityLUT.get(distance);

        subsystem.setTargetVelocity(Range.clip(velocity, 1100,1570));


    }
}
