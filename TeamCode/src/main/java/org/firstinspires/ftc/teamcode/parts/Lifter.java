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
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
        motor.setPower((gp2.left_trigger - gp2.right_trigger) * 0.5);
    }
}
