package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turner
{
    public Servo servo = null;

    public Turner(HardwareMap map)
    {
        servo = map.servo.get("turner");
    }

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
             if (gp2.dpad_left)  servo.setPosition(0.0);
        else if (gp2.dpad_right) servo.setPosition(1.0);
    }
}
