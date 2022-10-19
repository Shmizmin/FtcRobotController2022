package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lifter
{
    public DcMotor motor;

    public Lifter(HardwareMap map)
    {
        motor = map.dcMotor.get("lifter");
    }

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
             if (gp2.dpad_up)   motor.setPower(-0.8);
        else if (gp2.dpad_down) motor.setPower(0.8);
        else                    motor.setPower(0.0);
    }
}
