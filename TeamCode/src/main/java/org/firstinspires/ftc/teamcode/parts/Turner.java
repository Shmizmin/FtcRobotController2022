package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turner
{
    public CRServo servo = null;
    private int check = 1;

    public Turner(HardwareMap map)
    {
        servo = map.get(CRServo.class, "turner");
    }

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
        //if (gp2.right_stick_x < -0.05 ||
        //    gp2.right_stick_x > +0.05)
        //{
        //check++;

        if (gp2.dpad_left)
        {
            servo.setPower(-0.25);
        }

        else if (gp2.dpad_right)
        {
            servo.setPower(0.25);
        }

        else
        {
            servo.setPower(0.0);
        }

        try
        {
            Thread.sleep(50);
        }

        catch (InterruptedException ignored)
        {
        }

        //}

        //servo.setPower(gp2.right_stick_x * 0.5);
    }
}
