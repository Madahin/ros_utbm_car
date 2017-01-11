#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <chrono>
#include <array>

#include <QString>

long GetEpoch();
std::array<long, 2> GetSplitedEpoch();
std::array<double, 2> ConvertNMEAToDegree(QString lat, QChar latDir, QString lon, QChar lonDir);
void AppendDirSeparator(std::string& path);
void DeleteDirSeparator(std::string& path);

#endif
