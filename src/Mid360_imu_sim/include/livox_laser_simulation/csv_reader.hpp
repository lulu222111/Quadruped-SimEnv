//
// Created by lfc on 2021/3/1.
//

#ifndef SRC_GAZEBO_CSV_READER_HPP
#define SRC_GAZEBO_CSV_READER_HPP

#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class CsvReader {
 public:
    static bool ReadCsvFile(const std::string& file_name,
                            std::vector<std::vector<double>>& datas) {
        std::ifstream file_stream(file_name);
        if (!file_stream.is_open()) {
            std::cerr << "cannot read csv file: " << file_name << "\n";
            return false;
        }

        datas.clear();

        std::string header;
        if (!std::getline(file_stream, header)) {
            std::cerr << "csv file has no header: " << file_name << "\n";
            return false;
        }

        std::string line_str;
        std::size_t line_number = 1;

        while (std::getline(file_stream, line_str)) {
            ++line_number;

            // Strip trailing carriage return for CRLF compatibility
            if (!line_str.empty() && line_str.back() == '\r') {
                line_str.pop_back();
            }

            if (line_str.empty()) {
                std::cerr << "empty csv row at line " << line_number << "\n";
                return false;
            }

            std::stringstream line_stream(line_str);
            std::vector<double> data;
            std::string value;

            try {
                while (std::getline(line_stream, value, ',')) {
                    if (value.empty()) {
                        std::cerr << "empty csv value at line "
                                  << line_number << "\n";
                        return false;
                    }

                    std::size_t converted = 0;
                    const double number = std::stod(value, &converted);

                    if (converted != value.size()) {
                        std::cerr << "invalid csv value at line "
                                  << line_number << ": " << value << "\n";
                        return false;
                    }

                    if (!std::isfinite(number)) {
                        std::cerr << "non-finite csv value at line "
                                  << line_number << ": " << value << "\n";
                        return false;
                    }

                    data.push_back(number);
                }
            } catch (const std::exception& error) {
                std::cerr << "cannot parse csv line " << line_number
                          << ": " << line_str
                          << " (" << error.what() << ")\n";
                return false;
            }

            if (data.size() != 3) {
                std::cerr << "csv line " << line_number
                          << " has " << data.size()
                          << " fields instead of 3\n";
                return false;
            }

            datas.push_back(data);
        }

        if (file_stream.bad() || datas.empty()) {
            std::cerr << "failed to read csv data: " << file_name << "\n";
            return false;
        }

        std::cerr << "data size:" << datas.size() << "\n";
        return true;
    }
};

#endif  // SRC_GAZEBO_CSV_READER_HPP
