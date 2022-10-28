package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lifter
{
    public DcMotor motor;

    public Lifter(HardwareMap map)
    {
        motor = map.get(DcMotor.class, "lifter");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
        motor.setPower((gp2.right_trigger - gp2.left_trigger) * 0.5);
    }
}
