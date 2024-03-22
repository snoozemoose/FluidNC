// Copyright (c) 2020 -	Bart Dring
// Copyright (c) 2020 -	Stefan de Bruijn
// Copyright (c) 2024 -	Adam Popanda
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    BD600.cpp

    This is for a Folinn Bd600 VFD based spindle via RS485 Modbus.
                         WARNING!!!!
    VFDs are very dangerous. They have high voltages and are very powerful
    Remove power before changing bits.

    ==============================================================================

    If a user changes state or RPM level, the command to do that is sent. If
    the command is not responded to a message is sent to serial that there was
    a timeout. If the system is in a critical state, an alarm will be generated and
    the machine stopped.

    If there are no commands to execute, various status items will be polled. If there
    is no response, it will behave as described above. It will stop any running jobs with
    an alarm.

    ===============================================================================

    Protocol Details
    The manual for the BD600 contains an Appendix A that talks about the details for communicating with the spindle.
    Link to the manual below:
    https://cononmotor.com.au/wp-content/uploads/2017/09/BD600-Manual.pdf

    Before using spindle, VFD must be setup for RS485 and match your spindle:

    F00.01 -> 2 -> RS485 command source
    F00.03 -> 400 -> maximum frequency in Hz
    F00.05 -> 100 -> minimum frequency in Hz
    F00.06 -> 9 -> frequency A source to RS485
    F13.00 -> 1 -> RS485 local address on the VFD
    F13.01 -> 5 -> Baud rate, 9600
    F13.02 -> 3 -> Data format, 8N1
    F13.05 -> 1 -> Standard Modbus protocol

    =========================================================================

    Commands
    ADDR    CMD   ADR1   ADR2  DATA1   DATA2    CRC
    0x01    0x06  0x10   0x00   0x00    0x01    CRC                Start spindle clockwise
    0x01    0x06  0x10   0x00   0x00    0x05    CRC                Stop spindle
    0x01    0x06  0x10   0x00   0x00    0x02    CRC                Start spindle counter-clockwise

    ==========================================================================

    Setting RPM
    ADDR    CMD     LEN     DATA        CRC
    0x01    0x05    0x02    0x09 0xC4   0xBF 0x0F               Write Frequency (0x9C4 = 2500 = 25.00HZ)

    Response is same as data sent

    ==========================================================================

    Setting registers
    Addr    Read    Len     Reg     DataH   DataL   CRC     CRC
    0x01    0x01    0x03    5       0x00    0x00    CRC     CRC     //  PD005
    0x01    0x01    0x03    11      0x00    0x00    CRC     CRC     //  PD011
    0x01    0x01    0x03    143     0x00    0x00    CRC     CRC     //  PD143
    0x01    0x01    0x03    144     0x00    0x00    CRC     CRC     //  PD144

    Message is returned with requested value = (DataH * 16) + DataL (see decimal offset above)

    ==========================================================================

    Status registers
    Addr    Read    Len     Reg     DataH   DataL   CRC     CRC
    0x01    0x04    0x03    0x00    0x00    0x00    CRC     CRC     //  Set Frequency * 100 (25Hz = 2500)
    0x01    0x04    0x03    0x01    0x00    0x00    CRC     CRC     //  Ouput Frequency * 100
    0x01    0x04    0x03    0x02    0x00    0x00    CRC     CRC     //  Ouput Amps * 10
    0x01    0x04    0x03    0x03    0x00    0x00    0xF0    0x4E    //  Read RPM (example CRC shown)
    0x01    0x04    0x03    0x0     0x00    0x00    CRC     CRC     //  DC voltage
    0x01    0x04    0x03    0x05    0x00    0x00    CRC     CRC     //  AC voltage
    0x01    0x04    0x03    0x06    0x00    0x00    CRC     CRC     //  Cont
    0x01    0x04    0x03    0x07    0x00    0x00    CRC     CRC     //  VFD Temp
    
    Message is returned with requested value = (DataH * 16) + DataL (see decimal offset above)

    ==========================================================================

    The math:

        PD005   400  Maximum frequency Hz (Typical for spindles)
        PD011   120  Min Speed (Recommend Aircooled=120 Water=100)
        PD143   2    Poles most are 2 (I think this is only used for RPM calc from Hz)
        PD144   3000 Max rated motor revolution at 50 Hz => 24000@400Hz = 3000@50HZ

    During initialization these 4 are pulled from the VFD registers. It then sets min and max RPM
    of the spindle. So:

        MinRPM = PD011 * PD144 / 50 = 120 * 3000 / 50 = 7200 RPM min
        MaxRPM = PD005 * PD144 / 50 = 400 * 3000 / 50 = 24000 RPM max

    If you then set 12000 RPM, it calculates the frequency:

        int targetFrequency = targetRPM * PD005 / MaxRPM = targetRPM * PD005 / (PD005 * PD144 / 50) = 
                              targetRPM * 50 / PD144 = 12000 * 50 / 3000 = 200

    If the frequency is -say- 25 Hz, Huanyang wants us to send 2500 (eg. 25.00 Hz).
*/

#include "BD600Spindle.h"

#include <algorithm>  // std::max

namespace Spindles {
    BD600Spindle::BD600Spindle() : VFD() {
        // Baud rate is set in the F13.01 setting.  If it is not 9600, add, for example,
        // _baudrate = 19200;
    }

    void BD600Spindle::direction_command(SpindleState mode, ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 6;
        data.rx_length = 6;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x06; //write
        data.msg[2] = 0x10; //high-order address
        data.msg[3] = 0x00; //low-order address
        data.msg[4] = 0x00; //high-order data

        switch (mode) {
            case SpindleState::Cw:
                data.msg[5] = 0x01;
                break;
            case SpindleState::Ccw:
                data.msg[5] = 0x02;
                break;
            default:  // SpindleState::Disable
                data.msg[5] = 0x05;
                break;
        }
    }

    void IRAM_ATTR BD600Spindle::set_speed_command(uint32_t dev_speed, ModbusCommand& data) {
        if (dev_speed != 0 && (dev_speed < _minFrequency || dev_speed > _maxFrequency)) {
            log_warn(name() << " requested freq " << (dev_speed) << " is outside of range (" << _minFrequency << "," << _maxFrequency << ")");
        }

        // the inverter expects a value in percentage relative to the _maxFrequency set in F00.03
        uint32_t speed_percentage = (dev_speed / _maxFrequency) * 100 * 100; // percenage, two decimal places

        data.tx_length = 6;
        data.rx_length = 6;
       
        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x06;  // write
        data.msg[2] = 0x30;  // communication setting with source set to Frequency
        data.msg[3] = 0x00;
        data.msg[4] = speed_percentage >> 8;
        data.msg[5] = speed_percentage & 0xFF;
    }

    // This gets data from the VFS. It does not set any values
    VFD::response_parser BD600Spindle::initialization_sequence(int index, ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 6;
        data.rx_length = 6;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x01;  // Read setting
        data.msg[2] = 0x03;  // Len
        //      [3] = set below...
        data.msg[4] = 0x00;
        data.msg[5] = 0x00;

        switch (index) {
            case -1:
                data.msg[3] = 5;  // PD005: max frequency the VFD will allow. Normally 400.

                return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                    uint16_t value = (response[4] << 8) | response[5];

                    // Set current RPM value? Somewhere?
                    auto bd600Spindle           = static_cast<BD600Spindle*>(vfd);
                    bd600Spindle ->_maxFrequency = value;
                    return true;
                };
                break;
            case -2:
                data.msg[3] = 11;  // PD011: frequency lower limit. Normally 0.

                return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                    uint16_t value = (response[4] << 8) | response[5];

                    // Set current RPM value? Somewhere?
                    auto bd600Spindle            = static_cast<BD600Spindle*>(vfd);
                    bd600Spindle ->_minFrequency = value;

                    log_info(bd600Spindle ->name() << " PD0011, PD005 Freq range (" << (bd600Spindle ->_minFrequency / 100) << ","
                                              << (bd600Spindle ->_maxFrequency / 100) << ") Hz"
                                              << " (" << (bd600Spindle ->_minFrequency / 100 * 60) << "," << (bd600Spindle ->_maxFrequency / 100 * 60)
                                              << ") RPM");

                    return true;
                };
                break;
            case -3:
                data.msg[3] = 144;  // PD144: max rated motor revolution at 50Hz => 24000@400Hz = 3000@50HZ

                return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                    uint16_t value = (response[4] << 8) | response[5];

                    // Set current RPM value? Somewhere?
                    auto bd600Spindle            = static_cast<BD600Spindle*>(vfd);
                    bd600Spindle ->_maxRpmAt50Hz = value;

                    log_info(bd600Spindle ->name() << " PD144 Rated RPM @ 50Hz:" << bd600Spindle ->_maxRpmAt50Hz);

                    // Regarding PD144, the 2 versions of the manuals both say "This is set according to the
                    // actual revolution of the motor. The displayed value is the same as this set value. It
                    // can be used as a monitoring parameter, which is convenient to the user. This set value
                    // corresponds to the revolution at 50Hz".

                    // Calculate the VFD settings:
                    bd600Spindle ->updateRPM();

                    return true;
                };
                break;
            case -4:
                data.rx_length = 5;
                data.msg[3]    = 143;  // PD143: 4 or 2 poles in motor. Default is 4. A spindle being 24000RPM@400Hz implies 2 poles

                return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                    uint8_t value    = response[4];  // Single byte response.
                    auto    bd600Spindle  = static_cast<BD600Spindle*>(vfd);
                    // Sanity check. We expect something like 2 or 4 poles.
                    if (value <= 4 && value >= 2) {
                        // Set current RPM value? Somewhere?

                        bd600Spindle ->_numberPoles = value;

                        log_info(bd600Spindle ->name() << " PD143 Poles:" << bd600Spindle ->_numberPoles);

                        bd600Spindle ->updateRPM();

                        return true;
                    } else {
                        log_error(bd600Spindle ->name() << "  PD143 Poles: expected 2-4, got:" << value);
                        return false;
                    }
                };
                break;
            case -5:
                data.msg[3] = 14;  // Accel value displayed is X.X

                return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                    uint16_t value = (response[4] << 8) | response[5];

                    auto bd600Spindle  = static_cast<BD600Spindle*>(vfd);
                    log_info(bd600Spindle ->name() << " PD014 Accel:" << float(value) / 10.0);
                    return true;
                };
                break;
            case -6:
                data.msg[3] = 15;  // Decel alue displayed is X.X

                return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                    uint16_t value = (response[4] << 8) | response[5];

                    auto bd600Spindle  = static_cast<BD600Spindle*>(vfd);
                    log_info(bd600Spindle ->name() << " PD015 Decel:" << float(value) / 10.0);
                    return true;
                };
                break;
            default:
                break;
        }

        // Done.
        return nullptr;
    }

    void BD600Spindle::updateRPM() {
        /*
        PD005 = 400.00 ; max frequency the VFD will allow
        MaxRPM = PD005 * 60; but see PD176

        Frequencies are expressed in centiHz.
        */

        if (_minFrequency > _maxFrequency) {
            _minFrequency = _maxFrequency;
        }
        if (_speeds.size() == 0) {
            // Convert from Frequency in centiHz (the divisor of 100) to RPM (the factor of 60)
            SpindleSpeed minRPM = _minFrequency * 60 / 100;
            SpindleSpeed maxRPM = _maxFrequency * 60 / 100;
            shelfSpeeds(minRPM, maxRPM);
        }
        setupSpeeds(_maxFrequency);
        _slop = std::max(_maxFrequency / 40, 1);
    }

    VFD::response_parser BD600Spindle::get_status_ok(ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 6;
        data.rx_length = 6;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x04;
        data.msg[2] = 0x03;
        data.msg[3] = reg;
        data.msg[4] = 0x00;
        data.msg[5] = 0x00;

        if (reg < 0x03) {
            reg++;
        } else {
            reg = 0x00;
        }
        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool { return true; };
    }

    VFD::response_parser BD600Spindle::get_current_speed(ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 6;
        data.rx_length = 6;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x04;
        data.msg[2] = 0x03;
        data.msg[3] = 0x01;  // Output frequency
        data.msg[4] = 0x00;
        data.msg[5] = 0x00;

        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
            uint16_t frequency = (response[4] << 8) | response[5];

            // Store speed for synchronization
            vfd->_sync_dev_speed = frequency;
            return true;
        };
    }

    // Configuration registration
    namespace {
        SpindleFactory::InstanceBuilder<BD600Spindle> registration("BD600");
    }
}
