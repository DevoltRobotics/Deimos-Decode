package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;

@Config
@Autonomous(preselectTeleOp = "TeleOpRed")
public class spindex_test extends OpModeCommand {
    GamepadEx gamepadEx1;

    public spindex_test() {
        super(Alliance.ANY);
    }

    @Override
    public void initialize() {

        new RunCommand(()->{
           telemetry.update();
        }).schedule();

        gamepadEx1 = new GamepadEx(gamepad1);



    }
}
