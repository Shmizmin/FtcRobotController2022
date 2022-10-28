package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turner
{
    public CRServo servo = null;

    public Turner(HardwareMap map)
    {
        servo = map.get(CRServo.class, "turner");
    }

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
        if (gp2.right_stick_x < -0.05 ||
            gp2.right_stick_x > +0.05)
        {
            servo.setPower(gp2.right_stick_x * 0.25);
        }

        //servo.setPower(gp2.right_stick_x * 0.5);
    }
}
