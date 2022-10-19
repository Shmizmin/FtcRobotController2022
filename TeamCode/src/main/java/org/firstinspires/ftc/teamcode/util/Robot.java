package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot
{
    public MecanumBot bot = null;
    public Lifter lifter = null;
    //add other hardware here

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
        bot.onUpdate(gp1, gp2);
        lifter.onUpdate(gp1, gp2);
    }

    public Robot(HardwareMap map)
    {
        bot = new MecanumBot(map);
        lifter = new Lifter(map);
    }
}
