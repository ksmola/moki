#include "moki.h"
#include <Arduino.h>
#include "Globals.h"

moki::moki()
{
}

String moki::GetICCID()
{
    return HologramCloud.getICCID();
}

String moki::GetIMEI()
{
    return HologramCloud.getIMEI();
}

bool moki::CloudConnect() //non blocking, returns true if request is received
{
    return HologramCloud.connect();
}

bool moki::IsConnected()
{
    return HologramCloud.isConnected();
}

String moki::ConnectionStatus()
{
    switch (HologramCloud.getConnectionStatus())
    {
    case CLOUD_REGISTERED:
        return "No packet switched connection";
        break;
    case CLOUD_CONNECTED:
        return "Packet switched connection established";
        break;
    case CLOUD_ERR_UNAVAILABLE:
        return "Could not communicate with the modem";
        break;
    case CLOUD_ERR_SIM:
        return "A valid SIM card was not found";
        break;
    case CLOUD_ERR_UNREGISTERED:
        return "Could not register on the network";
        break;
    case CLOUD_ERR_SIGNAL:
        return "No tower was found";
        break;
    case CLOUD_ERR_CONNECT:
        return "SIM card is not active";
        break;
    case CLOUD_ERR_MODEM_OFF:
        return "Modem is powered off and cannot respond";
        break;
    default:
        break;
    }
}

String moki::ChargerStatus()
{
    switch (HologramCloud.getChargeState())
    {
    case CHARGE_STATUS_FAULT:
        return "Charger is in a fault state";
        break;
    case CHARGE_STATUS_INVALID1:
        return "Charger is in an invalid state";
        break;
    case CHARGE_STATUS_CHARGING:
        return "Charger is charging the battery";
        break;
    case CHARGE_STATUS_LOW_BATTERY:
        return "Charger has detected a low battery condition";
        break;
    case CHARGE_STATUS_CHARGED:
        return "Battery is fully charged";
        break;
    case CHARGE_STATUS_INVALID5:
        return "Charger is in an invalid state(5)";
        break;
    case CHARGE_STATUS_NO_BATTERY:
        return "No battery is connected";
        break;
    case CHARGE_STATUS_NO_INPUT:
        return "Powered by battery only";
        break;
    default:
        break;
    }
}

int moki::GetSignalStrength() // returns signal strength 0-4
{
    // 0: -113 dBm or less
    // 1: -111 dBm
    // 2 to 30: -109 to -53 dBm with 2 dBm steps
    // 31 to 98 – -51 dBm or greater
    // 99 – No signal
    int _signal;
    _signal = HologramCloud.getSignalStrength();

    if (_signal == 0)
    {
        return 1;
    }
    else if (_signal == 1)
    {
        return 2;
    }
    else if (_signal >= 2 && _signal <= 30)
    {
        return 3;
    }
    else if (_signal >= 31 && _signal <= 98)
    {
        return 4;
    }
    else if (_signal == 99)
    {
        return 0;
    }
}

bool moki::SendMessage(const char *_content, const char *_tag)
{
    return HologramCloud.sendMessage(_content, _tag);
}

bool moki::RGBon(int color)
{
    return HologramCloud.setRGB(color);
}

bool moki::RGBoff()
{
    rgbblink = false;
    return HologramCloud.offRGB();
}

bool moki::RGBblink(int color, int ms)
{
    unsigned long _currentMillis = millis();
    rgbblink = true;

    if (_currentMillis - _prevMillis >= ms)
    {
        _prevMillis = _currentMillis;

        if (_status == false)
        {
            HologramCloud.setRGB(color);
            _status = true;
        }
        else
        {
            HologramCloud.offRGB();
            _status = false;
        }
    }
}