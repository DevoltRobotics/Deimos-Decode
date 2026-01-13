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

        VelocityLUT.add(56, 1100);
        VelocityLUT.add(75, 1115);
        VelocityLUT.add(101, 1210);
        VelocityLUT.add(134, 1380);
        VelocityLUT.add(168, 1560); // new
        VelocityLUT.add(211, 1640);

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
