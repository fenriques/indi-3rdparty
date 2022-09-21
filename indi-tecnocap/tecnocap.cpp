#include <cstring>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <libindi/indicom.h>
#include <libindi/connectionplugins/connectionserial.h>

#include "config.h"
#include "tecnocap.h"

// We declare an auto pointer to TecnoCap.
static std::unique_ptr<TecnoCap> tecnocap(new TecnoCap());

#define FLAT_TIMEOUT 3
#define MAX_BUFF 22

TecnoCap::TecnoCap() : INDI::LightBoxInterface(this, true)
{
    //setVersion(TECNOCAP_VERSION_MAJOR, TECNOCAP_VERSION_MINOR);
}

const char *TecnoCap::getDefaultName()
{
    return "TecnoSky Tecnocap Driver";
}

bool TecnoCap::initProperties()
{
    INDI::DefaultDevice::initProperties();

    // Status
    IUFillText(&StatusT[0], "Cover", "Cover", nullptr);
    IUFillText(&StatusT[1], "Light", "Light", nullptr);
    IUFillTextVector(&StatusTP, StatusT, 2, getDeviceName(), "Status", "Status", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // Firmware version
    IUFillText(&FirmwareT[0], "Version", "Version", nullptr);
    IUFillTextVector(&FirmwareTP, FirmwareT, 1, getDeviceName(), "Firmware", "Firmware", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // Motor Step count
    int motor_steps_default = 48;
    IUFillNumber(&MotorStepsN[0], "MOTOR_STEPS_VALUE", "Motor Steps", "%.f", 0, 1000, 1, motor_steps_default);
    IUFillNumberVector(&MotorStepsNP, MotorStepsN, 1, getDeviceName(), "MOTOR_STEPS_VALUE", "Number of steps", OPTIONS_TAB,
                       IP_RW, 60, IPS_IDLE);

    // Gear Ratio
    int gear_ratio_default = 120;
    IUFillNumber(&GearRatioN[0], "GEAR_RATIO_VALUE", "Steps/Deg", "%.f", 0, 1000, 1, gear_ratio_default);
    IUFillNumberVector(&GearRatioNP, GearRatioN, 1, getDeviceName(), "GEAR_RATIO_VALUE", "Gear Ratio", OPTIONS_TAB,
                       IP_RW, 60, IPS_IDLE);

    /* Has Limits switch */
    IUFillSwitch(&HasLimitS[0], "YES", "YES", ISS_ON);
    IUFillSwitch(&HasLimitS[1], "NO", "NO", ISS_OFF);
    IUFillSwitchVector(&HasLimitSP, HasLimitS, 2, getDeviceName(), "HAS_LIMIT", "Limits Switch", OPTIONS_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    /* Motor Invert Direction */
    IUFillSwitch(&InvertDirectionS[0], "YES", "YES", ISS_ON);
    IUFillSwitch(&InvertDirectionS[1], "NO", "NO", ISS_OFF);
    IUFillSwitchVector(&InvertDirectionSP, InvertDirectionS, 2, getDeviceName(), "INVERT_DIRECTION", "Invert Direction", OPTIONS_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);


    // Open position steps
    int open_steps_default = gear_ratio_default*motor_steps_default/90;
    IUFillNumber(&OpenStepsN[0], "OPEN_STEPS_VALUE", "Open position", "%.f", 0, 10000, 1, open_steps_default);
    IUFillNumberVector(&OpenStepsNP, OpenStepsN, 1, getDeviceName(), "OPEN_STEPS_VALUE", "Steps", OPTIONS_TAB,
                       IP_RW, 60, IPS_IDLE);
    
    /* Stop */
    IUFillSwitch(&CoverHaltS[0], "HALT", "Halt", ISS_ON);
    IUFillSwitchVector(&CoverHaltSP, CoverHaltS, 1, getDeviceName(), "HALT_COVER", "Dust Cover", MAIN_CONTROL_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

   
    /* Calibration controls */
    IUFillSwitch(&CalibrationControlS[0], "OPEN", "Open", ISS_ON);
    IUFillSwitch(&CalibrationControlS[1], "CLOSE", "Close", ISS_OFF);
    IUFillSwitchVector(&CalibrationControlSP, CalibrationControlS, 2, getDeviceName(), "CALIBRATION_CONTROLS", "Motion", "Cover Calibration", IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    /* Calibration set position */
    IUFillSwitch(&CalibrationSetS[0], "OPEN", "Set Open", ISS_ON);
    IUFillSwitch(&CalibrationSetS[1], "CLOSE", "Set Close", ISS_OFF);
    IUFillSwitchVector(&CalibrationSetSP, CalibrationSetS, 2, getDeviceName(), "CALIBRATION_SET", "Set position", "Cover Calibration", IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);



    initDustCapProperties(getDeviceName(), MAIN_CONTROL_TAB);
    initLightBoxProperties(getDeviceName(), MAIN_CONTROL_TAB);

    LightIntensityN[0].min  = 0;
    LightIntensityN[0].max  = 100;
    LightIntensityN[0].step = 5;


    setDriverInterface(AUX_INTERFACE| LIGHTBOX_INTERFACE );
    addAuxControls();

    serialConnection = new Connection::Serial(this);
    serialConnection->registerHandshake([&]() { return Handshake(); });
   
    registerConnection(serialConnection);

    return true;
}


void TecnoCap::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);

    loadConfig(true, "MOTOR_STEPS_VALUE");
    loadConfig(true, "GEAR_RATIO_VALUE");
    loadConfig(true, "OPEN_STEPS_VALUE");
    loadConfig(true, "HAS_LIMIT");
    loadConfig(true, "INVERT_DIRECTION");

    isGetLightBoxProperties(dev);

}

bool TecnoCap::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&ParkCapSP);
        defineProperty(&CoverHaltSP);
        defineProperty(&LightSP);
        defineProperty(&LightIntensityNP);
        defineProperty(&StatusTP);
        defineProperty(&FirmwareTP);
        defineProperty(&MotorStepsNP);
        defineProperty(&GearRatioNP);
        defineProperty(&HasLimitSP);
        defineProperty(&InvertDirectionSP);
        defineProperty(&OpenStepsNP);
        defineProperty(&CalibrationControlSP);
        defineProperty(&CalibrationSetSP);
        
        getFirmwareVersion();
        getCoverStatus();
        updateDeviceConfiguration();
        updateMotorConfiguration();
        
        updateLightBoxProperties();

    }
    else
    {
        deleteProperty(ParkCapSP.name);
        deleteProperty(CoverHaltSP.name);
        deleteProperty(LightSP.name);
        deleteProperty(LightIntensityNP.name);
        deleteProperty(StatusTP.name);
        deleteProperty(FirmwareTP.name);
        deleteProperty(MotorStepsNP.name);
        deleteProperty(GearRatioNP.name);
        deleteProperty(HasLimitSP.name);
        deleteProperty(InvertDirectionSP.name);
        deleteProperty(OpenStepsNP.name);
        deleteProperty(CalibrationControlSP.name);
        deleteProperty(CalibrationSetSP.name);

        updateLightBoxProperties();


    }

    return true;
}

bool TecnoCap::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    // Make sure it is for us.
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (processLightBoxNumber(dev, name, values, names, n))
        {
            return true;
        }

        //Update motor steps
        if (strcmp(name, MotorStepsNP.name) == 0)
        {
            MotorStepsNP.s = IPS_OK;
            IUUpdateNumber(&MotorStepsNP,values, names, n);

            IDSetNumber(&MotorStepsNP, nullptr);
            
            updateMotorConfiguration();
            
            return true;
        }

        //Update gear ratio
        if (strcmp(name, GearRatioNP.name) == 0)
        {
            GearRatioNP.s = IPS_OK;
            IUUpdateNumber(&GearRatioNP,values, names, n);

            IDSetNumber(&GearRatioNP, nullptr);
            updateMotorConfiguration();

            return true;
        }

        //Update open position steps
        if (strcmp(name, OpenStepsNP.name) == 0)
        {
            OpenStepsNP.s = IPS_OK;
            IUUpdateNumber(&OpenStepsNP,values, names, n);

            IDSetNumber(&OpenStepsNP, nullptr);
            
            updateDeviceConfiguration();

            return true;
        }
        
    }

    // Nobody has claimed this, so let the parent handle it
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool TecnoCap::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    // Make sure it is for us.
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (processDustCapSwitch(dev, name, states, names, n))
        {
            return true;
        }
        if (processLightBoxSwitch(dev, name, states, names, n))
        {
            return true;
        }
        // Has Limit switch
        if (!strcmp(name, HasLimitSP.name))
        {
            HasLimitSP.s = IPS_OK;
            IUUpdateSwitch(&HasLimitSP, states, names, n);
            IDSetSwitch(&HasLimitSP, nullptr);
            updateDeviceConfiguration();
        
            return true;
        }

        // Invert direction
        if (!strcmp(name, InvertDirectionSP.name))
        {
            InvertDirectionSP.s = IPS_OK;
            IUUpdateSwitch(&InvertDirectionSP, states, names, n);
            IDSetSwitch(&InvertDirectionSP, nullptr);
            updateDeviceConfiguration();
        
            return true;
        }
        // Stop Motion
        if (!strcmp(name, CoverHaltSP.name))
        {
            CoverHaltSP.s = IPS_OK;
            IUUpdateSwitch(&CoverHaltSP, states, names, n);
            IDSetSwitch(&CoverHaltSP, nullptr);
            
            CoverHalt();

            return true;
        }
        // Calibration Control
        if (!strcmp(name, CalibrationControlSP.name))
        {
            CalibrationControlSP.s = IPS_OK;
            IUUpdateSwitch(&CalibrationControlSP, states, names, n);
            IDSetSwitch(&CalibrationControlSP, nullptr);
            
            char response[MAX_BUFF] = {0};
            if (CalibrationControlS[0].s == ISS_ON && CalibrationControlS[1].s == ISS_OFF)
            {

                // Sends jog open command
                if (!sendCommand("%CCJMO#", response))
                    return false;

            }else if (CalibrationControlS[0].s == ISS_OFF && CalibrationControlS[1].s == ISS_ON)
            {
                // Sends jog close command
                if (!sendCommand("%CCJMC#", response))
                    return false;
                

            }

            return true;
        }

        // Calibration Set Position
        if (!strcmp(name, CalibrationSetSP.name))
        {
            CalibrationSetSP.s = IPS_OK;
            IUUpdateSwitch(&CalibrationSetSP, states, names, n);
            IDSetSwitch(&CalibrationSetSP, nullptr);
            
            char response[MAX_BUFF] = {0};
            if (CalibrationSetS[0].s == ISS_ON && CalibrationSetS[1].s == ISS_OFF)
            {

                // Sends set open position
                if (!sendCommand("%%CCJSO#", response))
                    return false;

            }else if (CalibrationSetS[0].s == ISS_OFF && CalibrationSetS[1].s == ISS_ON)
            {
                // Sends set close command
                if (!sendCommand("%CCJSC?????#", response))
                    return false;
                
                LOGF_INFO("Set close position: <%s>", response);

                string resSetClose;
                resSetClose =formatResponse(response,6,5);
                
                int num = std::stoi(resSetClose);
                OpenStepsN[0].value = num;
                
                IDSetNumber(&OpenStepsNP, nullptr);
                updateDeviceConfiguration();
            }

            return true;
        }
        
    }
    
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool TecnoCap::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    // Make sure it is for us.
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (processLightBoxText(dev, name, texts, names, n))
        {
            return true;
        }
    }

    // Nobody has claimed this, so let the parent handle it
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool TecnoCap::ISSnoopDevice(XMLEle *root)
{
    snoopLightBox(root);

    return INDI::DefaultDevice::ISSnoopDevice(root);
}

/*
    Even if the driver is already connected to serial,
    a connection command has to be issued to start
    the communication with the device.
    A default
*/
bool TecnoCap::Handshake()
{

    PortFD = serialConnection->getPortFD();

    /* Drop RTS */
    int i = 0;
    i |= TIOCM_RTS;
    if (ioctl(PortFD, TIOCMBIC, &i) != 0)
    {
        LOGF_ERROR("IOCTL error %s.", strerror(errno));
        return false;
    }

    i |= TIOCM_RTS;
    if (ioctl(PortFD, TIOCMGET, &i) != 0)
    {
        LOGF_ERROR("IOCTL error %s.", strerror(errno));
        return false;
    }
  
    // Connect to the device
    char response[MAX_BUFF]={0};
    if (!sendCommand("%CCATT#", response))
        return false;
    

    return true;
}

/*
    Returns the firmware (not driver) version formatted as M.mm
*/
bool TecnoCap::getFirmwareVersion()
{

    char response[MAX_BUFF] = {0};
    if (!sendCommand("%CCVER???#", response))
        return false;

    
    string majorVer = formatResponse(response,6,1);
    string minorVer = formatResponse(response,7,2);
    string version = majorVer + '.'+ minorVer;

    IUSaveText(&FirmwareT[0], version.c_str());
    IDSetText(&FirmwareTP, nullptr);


    return true;
}

void TecnoCap::TimerHit()
{
    if (!isConnected())
        return;

    getCoverStatus();

    // parking or unparking timed out, try again
    if (ParkCapSP.s == IPS_BUSY && !strcmp(StatusT[0].text, "Timed out"))
    {
        if (ParkCapS[0].s == ISS_ON)
            ParkCap();
        else
            UnParkCap();
    }

    SetTimer(getCurrentPollingPeriod());

}

/*
    Reads the status from the device triggered by TimerHit

*/
bool TecnoCap::getCoverStatus()
{
    char response[MAX_BUFF];
    
     if (isSimulation())
    {
        if (ParkCapSP.s == IPS_BUSY && --simulationWorkCounter <= 0)
        {
            ParkCapSP.s = IPS_OK;
            IDSetSwitch(&ParkCapSP, nullptr);
            simulationWorkCounter = 0;
        }

        if (ParkCapSP.s == IPS_BUSY)
        {
            response[6] = '2';
        }
        else
        {
            response[6] = '4';
            // Parked/Closed
            if (ParkCapS[CAP_PARK].s == ISS_ON)
                response[6] = '1';
            else
                response[6] = '3';
        }

        response[5] = (LightS[FLAT_LIGHT_ON].s == ISS_ON) ? '1' : '0';
    }
    else
    {
        if (!sendCommand("%CCCS??#", response))
            return false;
    }
    LOGF_DEBUG("Response Status: <%s>", response);

    string coverStatus;
    coverStatus = formatResponse(response,6,1);

    string switchStatus;
    switchStatus = formatResponse(response,5,1);

    int num = std::stoi(coverStatus);
    switch (num)
    {
            case 0:
                IUSaveText(&StatusT[0], "Unknown 0");
                break;

            case 1:
                IUSaveText(&StatusT[0], "Closed");
                if (ParkCapSP.s == IPS_BUSY || ParkCapSP.s == IPS_IDLE)
                {
                    IUResetSwitch(&ParkCapSP);
                    ParkCapS[0].s = ISS_ON;
                    ParkCapSP.s   = IPS_OK;
                    LOG_INFO("Cover closed.");
                    IDSetSwitch(&ParkCapSP, nullptr);
                }
                break;
            case 2:
                IUSaveText(&StatusT[0], "Moving");
                IUResetSwitch(&ParkCapSP);
                ParkCapS[0].s = ISS_ON;
                ParkCapSP.s   = IPS_BUSY;
                LOG_INFO("Cover moving.");
                IDSetSwitch(&ParkCapSP, nullptr);
                break;

            case 3:
                IUSaveText(&StatusT[0], "Open");
                if (ParkCapSP.s == IPS_BUSY || ParkCapSP.s == IPS_IDLE)
                {
                    IUResetSwitch(&ParkCapSP);
                    ParkCapS[1].s = ISS_ON;
                    ParkCapSP.s   = IPS_OK;
                    LOG_INFO("Cover open.");
                    IDSetSwitch(&ParkCapSP, nullptr);
                }
                break;

            case 4:
                IUSaveText(&StatusT[0], "Halt / Position unknown");
                break;
            case 5:
                IUSaveText(&StatusT[0], "Error");
                break;

    }
 
    
    IDSetText(&StatusTP, nullptr);
    return true;
}

/*
    Convert the response msg from SendCommand to a string
    pos = starting position for substring
    length = number of character

*/
string TecnoCap::formatResponse(char *response, int pos, int length)
{
    string tmpString = response;

    string subString = tmpString.substr (pos, length);

    return subString;
}

/*
    Sends command to the device.
    Return messages are stored in response
    # is the stop_char
*/
bool TecnoCap::sendCommand(const char *command, char *response)
{
    int nbytes_read = 0, nbytes_written = 0, tty_rc = 0;
    char res[8] = {0};
    LOGF_DEBUG("CMD <%s>", command);

    if (!isSimulation())
    {
        tcflush(PortFD, TCIOFLUSH);
        if ((tty_rc = tty_write_string(PortFD, command, &nbytes_written)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial write error: %s", errorMessage);
            return false;
        }
    }

    if (isSimulation())
    {
        strncpy(res, "OK#", 8);
        nbytes_read = 3;
    }
    else
    {
        if ((tty_rc = tty_nread_section(PortFD, response, MAX_BUFF, '#', FLAT_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial read error: %s", errorMessage);
            return false;
        }
    }

    LOGF_DEBUG("Response: <%s>", response);

    return true;
}

/*
    For this device the park command is %CCCC#
*/
IPState TecnoCap::ParkCap()
{
    if (isSimulation())
    {
        simulationWorkCounter = 3;
        return IPS_BUSY;
    }

    char response[MAX_BUFF];
    if (!sendCommand("%CCCC#", response))
        return IPS_ALERT;

    string parkResponse;
    parkResponse = formatResponse(response,0,5);

    if (strcmp(parkResponse.c_str(), "%CCCC#") == 0)
    {

        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}

/*
    For this device the unpark command is %CCCO#
    The cover status when unparked is 3
*/
IPState TecnoCap::UnParkCap()
{
    if (isSimulation())
    {
        simulationWorkCounter = 3;
        return IPS_BUSY;
    }

    char response[MAX_BUFF];
    if (!sendCommand("%CCCO#", response))
        return IPS_ALERT;

    string unParkResponse;
    unParkResponse = formatResponse(response,0,5);

    if (strcmp(response, "%CCCO#") == 0)
    {


        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}

/*
    %CCCH# halts the cover motion
    The cover status is 4
*/

IPState TecnoCap::CoverHalt()
{
    if (isSimulation())
    {
        simulationWorkCounter = 3;
        return IPS_BUSY;
    }

    char response[MAX_BUFF];
    if (!sendCommand("%CCCH#", response))
        return IPS_ALERT;

    return IPS_OK;
}

bool TecnoCap::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    IUSaveConfigNumber(fp, &MotorStepsNP);
    IUSaveConfigNumber(fp, &GearRatioNP);
    IUSaveConfigNumber(fp, &OpenStepsNP);
    IUSaveConfigSwitch(fp, &HasLimitSP);
    IUSaveConfigSwitch(fp, &InvertDirectionSP);

    return saveLightBoxConfigItems(fp);
}

/*
    When the limit switch, the motor direction or the open position are
    updated, the driver stores these values in the device firmware.
*/
bool TecnoCap::updateDeviceConfiguration()
{
    char command[MAX_BUFF] = {0};
    char response[MAX_BUFF] = {0};
    char cHasLimit;
    char cInvertDirection;

    if(HasLimitS[0].s == ISS_OFF && HasLimitS[1].s == ISS_ON){
        cHasLimit = 'N';
    }
    if(HasLimitS[0].s == ISS_ON && HasLimitS[1].s == ISS_OFF){
        cHasLimit = 'Y';
    } 
    if(InvertDirectionS[0].s == ISS_OFF && InvertDirectionS[1].s == ISS_ON){
        cInvertDirection = 'N';
    }
    if(InvertDirectionS[0].s == ISS_ON && InvertDirectionS[1].s == ISS_OFF){
        cInvertDirection = 'Y';
    } 

    snprintf(command, MAX_BUFF, "%%CCP%c%05d%c#", cHasLimit, static_cast<int32_t>(OpenStepsN[0].value), cInvertDirection);
    LOGF_INFO("Device %s", command);

    if (!sendCommand(command, response))
        return false;
    LOGF_DEBUG("Device configuration: %s", response);

    return true;
}

/*
    Sends the Motor Steps and Gear Ratio to the device
*/
bool TecnoCap::updateMotorConfiguration()
{
    char command[MAX_BUFF];
    char response[MAX_BUFF] ;

    snprintf(command, MAX_BUFF, "%%CCM%03d%03d#", static_cast<int32_t>(MotorStepsN[0].value), static_cast<int32_t>(GearRatioN[0].value));
    
    if (!sendCommand(command, response))
        return false;
    LOGF_DEBUG("Motor configuration: %s", response);

 
    return true;
}

