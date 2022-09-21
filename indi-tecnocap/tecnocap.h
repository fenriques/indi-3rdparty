#pragma once

#include <libindi/defaultdevice.h>
#include <libindi/indidustcapinterface.h>
#include <libindi/indilightboxinterface.h>
using std::string;

namespace Connection
{
    class Serial;
}


class TecnoCap : public INDI::DefaultDevice , public INDI::LightBoxInterface, public INDI::DustCapInterface
{
public:
    TecnoCap();
    virtual ~TecnoCap() = default;

    virtual const char *getDefaultName() override;

    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    virtual void ISGetProperties(const char *dev);
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    virtual bool ISSnoopDevice(XMLEle *root) override;

    //static void parkTimeoutHelper(void *context);
    //static void unparkTimeoutHelper(void *context);
    
protected:

    virtual bool saveConfigItems(FILE *fp) override;
    virtual void TimerHit() override;

    // DustCapInterface Park and UnPark methods
    virtual IPState ParkCap() override;
    virtual IPState UnParkCap() override;
    virtual IPState CoverHalt();

     // LightBoxInterface methods
    //virtual bool SetLightBoxBrightness(uint16_t value) override;
    //virtual bool EnableLightBox(bool enable) override;

private: 

    bool Handshake();
    bool sendCommand(const char *command, char *response);
    //bool getStartupData();
    //bool ping();
    bool getCoverStatus();
    bool getFirmwareVersion();
    string formatResponse(char *response, int pos, int length);
    bool updateDeviceConfiguration();
    bool updateMotorConfiguration();
    //bool getBrightness();

    //void parkTimeout();
    //int parkTimeoutID { -1 };

    //void unparkTimeout();
    //int unparkTimeoutID { -1 };


    // Status
    ITextVectorProperty StatusTP;
    IText StatusT[2] {};

    // Firmware version
    ITextVectorProperty FirmwareTP;
    IText FirmwareT[1] {};

    // Motor Step count
    INumber MotorStepsN[1];
    INumberVectorProperty MotorStepsNP;

    // Motor Resolution / Gear Ratio
    INumber GearRatioN[1];
    INumberVectorProperty GearRatioNP;

    // Has Limit Switch
    ISwitch HasLimitS[2];
    ISwitchVectorProperty HasLimitSP;

    // Motor Invert Direction
    ISwitch InvertDirectionS[2];
    ISwitchVectorProperty InvertDirectionSP;

    // Open Position Steps
    INumber OpenStepsN[1];
    INumberVectorProperty OpenStepsNP;

    // Stop Motion
    ISwitch CoverHaltS[1];
    ISwitchVectorProperty CoverHaltSP;

    // Calibration open / close
    ISwitch CalibrationControlS[2];
    ISwitchVectorProperty CalibrationControlSP;

    // Calibration set position
    ISwitch CalibrationSetS[2];
    ISwitchVectorProperty CalibrationSetSP;

    int PortFD{-1};
    uint16_t productID{ 0 };

    uint8_t simulationWorkCounter{ 0 };
    string prevCoverStatus;
    uint8_t prevLightStatus{ 0xFF };
    uint8_t prevMotorStatus{ 0xFF };
    uint8_t prevBrightness{ 0xFF };

    Connection::Serial *serialConnection{nullptr};
};
