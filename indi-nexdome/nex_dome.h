/*******************************************************************************
 Copyright(c) 2019 Jasem Mutlaq. All rights reserved.

 NexDome Driver for Firmware v3+

 Change Log:

 2019.10.07: Driver is completely re-written to work with Firmware v3 since
 Firmware v1 is obsolete from NexDome.
 2017.01.01: Driver for Firmware v1 is developed by Rozeware Development Ltd.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#pragma once

#include <indidome.h>

#include <math.h>
#include <map>
#include <string>
#include <sys/time.h>

#include "nex_dome_constants.h"

class NexDome : public INDI::Dome
{
    public:
        NexDome();

        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool initProperties() override;
        const char *getDefaultName() override;
        bool updateProperties() override;

    protected:
        bool Handshake() override;
        void TimerHit() override;

        // Motion
        virtual IPState MoveAbs(double az) override;
        virtual bool Sync(double az) override;

        // Shutter
        virtual IPState ControlShutter(ShutterOperation operation) override;

        // Abort
        virtual bool Abort() override;

        // Config
        virtual bool saveConfigItems(FILE * fp) override;

        // Parking
        virtual IPState Park() override;
        virtual IPState UnPark() override;
        virtual bool SetCurrentPark() override;
        virtual bool SetDefaultPark() override;

        /////////////////////////////////////////////////////////////////////////////
        /// Properties
        /////////////////////////////////////////////////////////////////////////////
        ISwitchVectorProperty OperationSP;
        ISwitch OperationS[1];
        enum
        {
            OP_HOME,
            OP_CALIBRATE,
        };

        INumberVectorProperty HomePositionNP;
        INumber HomePositionN[1];

        INumberVectorProperty BatteryLevelNP;
        INumber BatteryLevelN[2];

        ITextVectorProperty FirmwareVersionTP;
        IText FirmwareVersionT[1] {};

        ISwitchVectorProperty CloseShutterOnParkSP;
        ISwitch CloseShutterOnParkS[2];

        // Settings
        enum
        {
            ACCELERATION_RAMP,
            DEAD_ZONE,
            TRAVEL_RANGE,
            VELOCITY,
        } Settings;

        INumberVectorProperty RotatorSettingsNP;
        INumber RotatorSettingsN[4];

        INumberVectorProperty ShutterSettingsNP;
        INumber ShutterSettingsN[4];

    private:
        ///////////////////////////////////////////////////////////////////////////////
        /// Startup
        ///////////////////////////////////////////////////////////////////////////////
        bool getStartupValues();

        ///////////////////////////////////////////////////////////////////////////////
        /// Utility Functions
        ///////////////////////////////////////////////////////////////////////////////
        bool setParameter(ND::Commands command, ND::Targets target, int32_t value);
        bool getParameter(ND::Commands command, ND::Targets target, std::string value);
        bool checkEvents(std::string &response);
        bool processEvent(const std::string &event);
        bool sendCommand(const char * cmd, char * res = nullptr, int cmd_len = -1, int res_len = -1);
        void hexDump(char * buf, const char * data, int size);

        std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");
        std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");
        std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ");
        std::vector<std::string> split(const std::string &input, const std::string &regex);

};

