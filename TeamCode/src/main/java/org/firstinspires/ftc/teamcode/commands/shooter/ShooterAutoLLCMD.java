package org.firstinspires.ftc.teamcode.commands.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterAutoLLCMD extends CommandBase {

    private static InterpLUT VelocityLUT = new InterpLUT();

    static {
        VelocityLUT.add(0.000000000000000012, 1900);

        VelocityLUT.add(0.20, 1540);
        VelocityLUT.add(0.35, 1400);
        VelocityLUT.add(0.63, 1270); // new
        VelocityLUT.add(2, 1175);

        VelocityLUT.add(1000000000, 0);


        VelocityLUT.createLUT();
    }

    private final ShooterSubsystem subsystem;

    private final LLSubsystem LL;

    public ShooterAutoLLCMD(ShooterSubsystem subsystem, LLSubsystem LL) {
        this.subsystem = subsystem;
        this.LL = LL;  
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        LL.setAimingPipeline();

        if (LL.result != null && LL.result.isValid()) {
            Double Area = LL.getAllianceTA();
            if(Area != null){
                double velocity = VelocityLUT.get(Area);

                subsystem.setTargetVelocity(Range.clip(velocity, 1175,1540));
            }
        }
    }
}
