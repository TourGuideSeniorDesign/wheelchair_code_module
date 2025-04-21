//
// Created by Robbie on 4/21/25.
//

#ifndef WHEELCHAIR_CODE_MODULE_TEMP_MONITOR_H
#define WHEELCHAIR_CODE_MODULE_TEMP_MONITOR_H

std::string findCpuThermalZone();

double readTemperature(const std::string& tempPath);

#endif //WHEELCHAIR_CODE_MODULE_TEMP_MONITOR_H
