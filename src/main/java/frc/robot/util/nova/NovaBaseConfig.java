package frc.robot.util.nova;

import static com.thethriftybot.ThriftyNova.Error.*;
import static frc.robot.util.nova.NovaUtil.tryUntilOk;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.ExternalEncoder;
import com.thethriftybot.ThriftyNova.MotorType;
import com.thethriftybot.ThriftyNova.PIDSlot;
import java.util.HashMap;
import java.util.Map;

public abstract class NovaBaseConfig {
    protected int maxAttempts = 1;
    private Map<Integer, Object> parameters = new HashMap<>();

    protected void putParameter(int parameterId, Object value) {
        parameters.put(parameterId, value);
    }

    protected Object getParameter(int parameterId) {
        return parameters.get(parameterId);
    }

    protected Object getParameter(NovaBaseConfig fromConfig, int parameterId) {
        return fromConfig.getParameter(parameterId);
    }

    protected void removeParameter(int parameterId) {
        parameters.remove(parameterId);
    }

    protected void removeParameter(NovaBaseConfig fromConfig, int parameterId) {
        fromConfig.removeParameter(parameterId);
    }

    protected void apply(NovaBaseConfig config) {
        for (Map.Entry<Integer, Object> parameter : config.parameters.entrySet()) {
            putParameter(parameter.getKey(), parameter.getValue());
        }
    }

    protected void configure(ThriftyNova nova) {
        for (Map.Entry<Integer, Object> parameter : this.parameters.entrySet()) {
            switch (parameter.getKey()) {
                case 0 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.setInversion((boolean) parameter.getValue()), SET_INVERSION);
                case 1 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.setBrakeMode((boolean) parameter.getValue()), SET_BRAKE_MODE);
                case 2 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setMaxOutput((double) parameter.getValue(), (double) this.parameters.get(3)),
                        SET_MAX_FWD,
                        SET_MAX_REV);
                case 4 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.setRampUp((double) parameter.getValue()), SET_RAMP_UP);
                case 5 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.setRampDown((double) parameter.getValue()), SET_RAMP_DOWN);
                case 6 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setMaxCurrent(CurrentType.STATOR, (double) parameter.getValue()),
                        SET_MAX_CURRENT);
                case 7 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setMaxCurrent(CurrentType.SUPPLY, (double) parameter.getValue()),
                        SET_MAX_CURRENT);
                case 8 -> tryUntilOk(nova, maxAttempts, () -> nova.follow((int) parameter.getValue()), SET_FOLLOWER_ID);
                case 9 -> tryUntilOk(nova, maxAttempts, () -> nova.pid0.setP((double) parameter.getValue()), SET_KP_0);
                case 10 -> tryUntilOk(nova, maxAttempts, () -> nova.pid0.setI((double) parameter.getValue()), SET_KI_0);
                case 11 -> tryUntilOk(nova, maxAttempts, () -> nova.pid0.setD((double) parameter.getValue()), SET_KD_0);
                case 12 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.pid0.setFF((double) parameter.getValue()), SET_KF_0);
                case 13 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.pid0.setAccumulatorCap((double) parameter.getValue()),
                        SET_IZONE_0);
                case 14 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.pid0.setAllowableError((double) parameter.getValue()),
                        SET_PID_ERROR_0);
                case 15 -> tryUntilOk(nova, maxAttempts, () -> nova.pid1.setP((double) parameter.getValue()), SET_KP_1);
                case 16 -> tryUntilOk(nova, maxAttempts, () -> nova.pid1.setI((double) parameter.getValue()), SET_KI_1);
                case 17 -> tryUntilOk(nova, maxAttempts, () -> nova.pid1.setD((double) parameter.getValue()), SET_KD_1);
                case 18 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.pid1.setFF((double) parameter.getValue()), SET_KF_1);
                case 19 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.pid1.setAccumulatorCap((double) parameter.getValue()),
                        SET_IZONE_1);
                case 20 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.pid1.setAllowableError((double) parameter.getValue()),
                        SET_PID_ERROR_1);
                case 21 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.canFreq.setFault((double) parameter.getValue()), SET_FREQ_FAULT);
                case 22 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.canFreq.setSensor((double) parameter.getValue()),
                        SET_FREQ_SENSOR);
                case 23 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.canFreq.setQuadSensor((double) parameter.getValue()),
                        SET_FREQ_QUAD_SENSOR);
                case 24 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.canFreq.setControl((double) parameter.getValue()), SET_FREQ_CTRL);
                case 25 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.canFreq.setCurrent((double) parameter.getValue()),
                        SET_FREQ_CURRENT);
                case 26 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setSoftLimits(
                                (double) parameters.getOrDefault(27, -(double) parameter.getValue()),
                                (double) parameter.getValue()),
                        SET_SOFT_LIMIT_FWD,
                        SET_SOFT_LIMIT_REV);
                case 28 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.enableSoftLimits((boolean) parameter.getValue()),
                        ENABLE_SOFT_LIMIT);
                case 29 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.enableHardLimits((boolean) parameter.getValue()),
                        ENABLE_HARD_LIMIT);
                case 30 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.setAbsOffset((int) parameter.getValue()), SET_ABS_OFFSET);
                case 31 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setVoltageCompensation((double) parameter.getValue()),
                        SET_VOLTAGE_COMP);
                case 32 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setExternalEncoder((ExternalEncoder) parameter.getValue()),
                        EXTERNAL_ENCODER_SELECT);
                case 33 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setTempThrottleEnable((boolean) parameter.getValue()),
                        ENABLE_TEMP_THROTTLE);
                case 34 -> tryUntilOk(
                        nova, maxAttempts, () -> nova.setMotorType((MotorType) parameter.getValue()), SET_MOTOR_TYPE);
                case 36 -> tryUntilOk(
                        nova,
                        maxAttempts,
                        () -> nova.setAbsoluteWrapping((boolean) parameter.getValue()),
                        SET_ABSOLUTE_WRAPPING);
                case 37 -> nova.usePIDSlot((PIDSlot) parameter.getValue());
                case 38 -> nova.setNTLogging((boolean) parameter.getValue());
                case 39 -> nova.useEncoderType((EncoderType) parameter.getValue());
                case 40 -> nova.setEncoderPosition((double) parameter.getValue());
            }
        }
    }
}
