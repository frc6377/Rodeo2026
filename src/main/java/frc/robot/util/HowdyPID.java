// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;

/** Add your docs here. */
@SuppressWarnings("unused")
public class HowdyPID {
    private Consumer<Double> consumerP;
    private Consumer<Double> consumerI;
    private Consumer<Double> consumerD;
    private Consumer<Double> consumerS;
    private Consumer<Double> consumerV;
    private Consumer<Double> consumerA;
    private Consumer<Double> consumerG;

    private TunableNumber tuneP;
    private TunableNumber tuneI;
    private TunableNumber tuneD;
    private TunableNumber tuneS;
    private TunableNumber tuneV;
    private TunableNumber tuneA;
    private TunableNumber tuneG;

    private final SlotConfigs slotConfigs = new SlotConfigs();

    public HowdyPID() {
        setKP(0);
        setKI(0);
        setKD(0);
        setKS(0);
        setKV(0);
        setKA(0);
        setKG(0);
    }

    public HowdyPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        setKP(kP);
        setKI(kI);
        setKD(kD);
        setKS(kS);
        setKV(kV);
        setKA(kA);
        setKG(kG);
    }

    public HowdyPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG, GravityTypeValue GT) {
        setKP(kP);
        setKI(kI);
        setKD(kD);
        setKS(kS);
        setKV(kV);
        setKA(kA);
        setKG(kG);
        setGravityType(GT);
    }

    public void setKP(double kP) {
        slotConfigs.withKP(kP);
    }

    public void setKI(double kI) {
        slotConfigs.withKI(kI);
    }

    public void setKD(double kD) {
        slotConfigs.withKD(kD);
    }

    public void setKS(double kS) {
        slotConfigs.withKS(kS);
    }

    public void setKV(double kV) {
        slotConfigs.withKV(kV);
    }

    public void setKA(double kA) {
        slotConfigs.withKA(kA);
    }

    public void setKG(double kG) {
        slotConfigs.withKG(kG);
    }

    public void setGravityType(GravityTypeValue gravityTypeValue) {
        slotConfigs.withGravityType(gravityTypeValue);
    }

    public void setStaticFeedforwardSign(StaticFeedforwardSignValue staticFeedforwardSignValue) {
        slotConfigs.withStaticFeedforwardSign(staticFeedforwardSignValue);
    }

    public Slot0Configs getSlot0Configs() {
        return Slot0Configs.from(slotConfigs);
    }

    public Slot1Configs getSlot1Configs() {
        return Slot1Configs.from(slotConfigs);
    }

    public Slot2Configs geSlot2Configs() {
        return Slot2Configs.from(slotConfigs);
    }

    public void createTunableNumbers(String name, Subsystem subsystem) {
        consumerP = (value) -> setKP(value);
        consumerI = (value) -> setKI(value);
        consumerD = (value) -> setKD(value);
        consumerS = (value) -> setKS(value);
        consumerV = (value) -> setKV(value);
        consumerA = (value) -> setKA(value);
        consumerG = (value) -> setKG(value);

        tuneP = new TunableNumber(name + ": kP", slotConfigs.kP, consumerP, subsystem);
        tuneI = new TunableNumber(name + ": kI", slotConfigs.kI, consumerI, subsystem);
        tuneD = new TunableNumber(name + ": kD", slotConfigs.kD, consumerD, subsystem);
        tuneS = new TunableNumber(name + ": kS", slotConfigs.kS, consumerS, subsystem);
        tuneV = new TunableNumber(name + ": kV", slotConfigs.kV, consumerV, subsystem);
        tuneA = new TunableNumber(name + ": kA", slotConfigs.kA, consumerA, subsystem);
        tuneG = new TunableNumber(name + ": kG", slotConfigs.kG, consumerG, subsystem);
    }
}
