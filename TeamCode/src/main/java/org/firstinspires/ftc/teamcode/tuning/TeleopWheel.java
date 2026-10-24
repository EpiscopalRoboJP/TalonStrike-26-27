package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.Proxy;

/**
 * Road Runner sends positive power for a wheel rolling forward. Teleop forward
 * is negative power on every wheel, so tuner commands are negated here.
 */
final class TeleopWheel {
    private TeleopWheel() {
    }

    static DcMotorEx negated(DcMotorEx motor) {
        return (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class},
                (proxy, method, args) -> {
                    if ("setPower".equals(method.getName())) {
                        motor.setPower(-((Number) args[0]).doubleValue());
                        return null;
                    }
                    if ("getPower".equals(method.getName())) {
                        return -motor.getPower();
                    }
                    if (args == null || args.length == 0) {
                        return method.invoke(motor);
                    }
                    return method.invoke(motor, args);
                });
    }
}
