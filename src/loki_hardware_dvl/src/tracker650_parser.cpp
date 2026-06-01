#include "tracker650_ros_driver/tracker650_parser.hpp"
#include <iostream>
#include <sstream>
#include <exception>
#include <memory>

const char* Tracker650Parser::lastStatus() const
{
  return last_status_;
}


// Data format:
// $DVPDX,tu,dtu,adr,adp,ady,pdx,pdy,pdz,c,mode,pitch,roll,stdoff*hh
//
// Parameters:
// - line: raw DVL UDP sentence
// - delimiter: character used to split fields (usually ',')
//
// std::stringstream converts the raw string into a stream.
// std::getline() extracts one field at a time until the delimiter is hit.
//
// Output:
// Returns a vector where each comma-separated value becomes one element.
std::vector<std::string> Tracker650Parser::split(const std::string& line, char delimiter) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;

    while (std::getline(ss, field, delimiter)) {
        fields.push_back(field);
    }

    return fields;
}

std::string Tracker650Parser::formatStdoff(const std::string& field) {
    size_t star_pos = field.find('*');
    if (star_pos != std::string::npos) {
        return field.substr(0, star_pos);
    }
    return field;
}

bool Tracker650Parser::parseDVPDX(const std::string& line, DvlVelocity& out) {
    auto fields = split(line, ',');

    if (fields.empty() || fields[0] != "$DVPDX") {
        return false;
    }

    if (fields.size() < 14) {
        std::cerr << "Invalid DVPDX packet: " << line << "\n";
        return false;
    }

    try {
        double tu  = std::stod(fields[1]);
        double dtu = std::stod(fields[2]);
        double pdx = std::stod(fields[6]);
        double pdy = std::stod(fields[7]);
        double pdz = std::stod(fields[8]);

        int confidence = std::stoi(fields[9]);

        double pitch = std::stod(fields[11]);
        double roll  = std::stod(fields[12]);
        double standoff = std::stod(formatStdoff(fields[13]));

        if (confidence == 0) {
            std::cout << "DVPDX received, confidence=0, ignoring measurement\n";
            return false;
        }

        double dt_sec = dtu / 1e6;

        if (dt_sec <= 0.0) {
            std::cerr << "Invalid dtu, cannot calculate velocity: " << dtu << "\n";
            return false;
        }

        out.time = tu / 1e6;
        out.vx = pdx / dt_sec;
        out.vy = pdy / dt_sec;
        out.vz = pdz / dt_sec;
        out.confidence = confidence;

        std::cout
            << "DVPDX parsed | "
            << "vx=" << out.vx << " m/s, "
            << "vy=" << out.vy << " m/s, "
            << "vz=" << out.vz << " m/s, "
            << "confidence=" << confidence << ", "
            << "pitch=" << pitch << ", "
            << "roll=" << roll << ", "
            << "standoff=" << standoff << "\n";

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to parse DVPDX packet: " << e.what() << "\n";
        std::cerr << "Packet was: " << line << "\n";
        return false;
    }
}
