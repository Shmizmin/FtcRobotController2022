package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue Left")
public class BlueLeft extends AutoAbstract
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        onInit();

        turner.servo.setPosition(1.0);

        sleep(750);

        camera.webcam.stopStreaming();

        lifter.motor.setTargetPosition(-200);
        lifter.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.motor.setPower(0.6);
        sleep(250);

        //measure distance here for farther
        forward(1.5, 2.0);

        right(22.5, 6.0);

        forward(25.0, 3.0);

        left(11.5, 5.0);

        lifter.motor.setTargetPosition(-950);
        lifter.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.motor.setPower(0.6);
        sleep(500);

        forward(5.0, 3.0);

        turner.servo.setPosition(0.0);

        sleep(500);

        backward(4.5, 3.0);

        lifter.motor.setPower(0.0);
        lifter.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100);

        //telemetry.addData("Position: ", camera.scanner.position.toString());

        switch (camera.scanner.position)
        {
            case RED:
            {
                left(12.5 + 22.5, 6.0);
            } break;

            case GREEN:
            {
                left(12.5, 6.0);
            } break;

            case BLUE:
            {
                right(11.5, 5.0);
            } break;
        }
    }
}
