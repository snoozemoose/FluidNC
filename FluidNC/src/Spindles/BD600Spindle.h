// Copyright (c) 2020 -	Bart Dring
// Copyright (c) 2020 -	Stefan de Bruijn
// Copyright (c) 2022 -	Peter Newbery
// Copyright (c) 2024 -	Adam Popanda
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "VFDSpindle.h"

namespace Spindles {
    class BD600Spindle : public VFD {
    private:
        int reg;

    protected:
        uint16_t _minFrequency = 100;    
        uint16_t _maxFrequency = 400;  
        uint16_t _numberPoles  = 4; 
        uint16_t _NumberPhases  = 3;

        void updateRPM();

        void direction_command(SpindleState mode, ModbusCommand& data) override;
        void set_speed_command(uint32_t rpm, ModbusCommand& data) override;

        response_parser initialization_sequence(int index, ModbusCommand& data) override;
        response_parser get_current_speed(ModbusCommand& data) override;

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override { return "BD600"; }

    public:
        BD600Spindle();
    };
}
