package org.firstinspires.ftc.teamcode.SolomonRandom;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(name = "MaxBotix Range Sensor", description = "Ultrasonic Sensor from MaxBotix", xmlTag = "MaxBotixI2CRangeSensor")
public class SensorMBRangeSensor extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    //----------------------------------------------------------------------------------------------------
    // USER METHODS
    //----------------------------------------------------------------------------------------------------
    double startTime = 0;

    public double getRange(DistanceUnit unit)
    {
        return unit.fromCm(getRangeRaw());
    }

    //----------------------------------------------------------------------------------------------------
    // RAW REGISTER READS
    //----------------------------------------------------------------------------------------------------

    public short getRangeRaw()
    {
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 10) {}
        return TypeConversion.byteArrayToShort(deviceClient.read(2));
    }

    //----------------------------------------------------------------------------------------------------
    // READ AND WRITE METHODS
    //----------------------------------------------------------------------------------------------------

    public void writeRange() {
        deviceClient.write(TypeConversion.intToByteArray(0x51));
    }


    //----------------------------------------------------------------------------------------------------
    // REGISTER AND CONFIG SETTINGS
    //----------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------
    // CONSTRUCTION AND INITIALIZATION
    //----------------------------------------------------------------------------------------------------

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    public SensorMBRangeSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);

        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.valueOf("MaxBotix");
    }

    @Override
    public String getDeviceName()
    {
        return "MaxBotix Range Sensor";
    }


}
