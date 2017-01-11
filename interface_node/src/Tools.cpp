#include "include/Tools.hpp"

long GetEpoch()
{
    auto epoch_t = std::chrono::high_resolution_clock::now();
    auto epoch   = epoch_t.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(epoch).count();
}

std::array<long, 2> GetSplitedEpoch()
{
    const unsigned int pad = 10;
    std::array<long, 2> res;
    long epoch = GetEpoch();

    QString sepoch = QString::number(epoch);

    res[0] = sepoch.left(pad).toLong();
    res[1] = sepoch.right(sepoch.length()-pad).toLong();

    return res;
}

#include <iostream>
std::array<double, 2> ConvertNMEAToDegree(QString lat, QChar latDir, QString lon, QChar lonDir)
{
    std::array<double, 2> res = {0, 0};

    const int latDotPos = lat.indexOf('.');

    QString slatMin = lat.right(lat.length() - latDotPos + 2);
    QString slatDay = lat.left(lat.length() - slatMin.length());

    const int lonDotPos = lon.indexOf('.');

    QString slonMin = lon.right(lon.length() - lonDotPos + 2);
    QString slonDay = lon.left(lon.length() - slonMin.length());

    if(!slatDay.isEmpty()){
        res[0] = slatDay.toDouble();
    }
    res[0] = (res[0] + slatMin.toDouble() / 60.0) * ((latDir == 'S') ? -1 : 1);

    if(!slonDay.isEmpty()){
        res[1] = slonDay.toDouble();
    }
    res[1] = (res[1] + slonMin.toDouble() / 60.0) * ((lonDir == 'W') ? -1 : 1);

#ifndef NDEBUG
    std::cout << "lon : " << slonDay.toStdString() << "'" << slonMin.toStdString() << " -> " << res[1] << std::endl;
    std::cout << "lat : " << slatDay.toStdString() << "'" << slatMin.toStdString() << " -> " << res[0] << std::endl;
#endif

    return res;
}

void AppendDirSeparator(std::string& path)
{
    if(path.at(path.length()-1) != '/'){
        path.append("/");
    }
}

void DeleteDirSeparator(std::string& path)
{
    if(path.at(path.length()-1) = '/'){
        path.pop_back();
    }
}
