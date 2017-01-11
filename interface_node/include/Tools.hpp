#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <chrono>
#include <array>

#ifndef NDEBUG
#include <iostream>
#endif

#include <QString>

/**
 * @brief GetEpoch retreive the epoch in microsecond
 * @return the epoch
 */
long GetEpoch();

/**
 * @brief GetSplitedEpoch separate the second and the microsecond of GetEpoch()
 * @return an array containing the epoch in microseconds
 */
std::array<long, 2> GetSplitedEpoch();

/**
 * @brief ConvertNMEAToDegree convert NMEA longitude and latitude to degree
 * @param lat the latitude in degree minute secondes
 * @param latDir the direction 'N' or 'S'
 * @param lon the longitude in degree minute secondes
 * @param lonDir the direction 'E' or 'W'
 * @return an array containing a position in degree, {0, 0} if an error occured
 */
std::array<double, 2> ConvertNMEAToDegree(QString lat, QChar latDir, QString lon, QChar lonDir);

/**
 * @brief AppendDirSeparator make sure a path as a separator at the end
 * @param path the path to sanitize
 */
void AppendDirSeparator(std::string& path);

/**
 * @brief DeleteDirSeparator make sur a path as no separator as the end
 * @param path the path to sanitize
 */
void DeleteDirSeparator(std::string& path);

#endif
