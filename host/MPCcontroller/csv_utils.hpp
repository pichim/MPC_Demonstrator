
#pragma once

#include <Eigen/Dense>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>

// Remove carriage returns and surrounding whitespace.
inline std::string cleanCsvCell(const std::string& cell)
{
    const auto first = cell.find_first_not_of(" \t\r\n");

    if (first == std::string::npos) {
        return "";
    }

    const auto last = cell.find_last_not_of(" \t\r\n");

    return cell.substr(first, last - first + 1);
}

inline Eigen::MatrixXf loadMatrixCSV(const std::string& path)
{
    std::ifstream f(path);

    if (!f.is_open()) {
        throw std::runtime_error("cannot open " + path);
    }

    std::vector<std::vector<float>> rows;
    std::string line;

    while (std::getline(f, line)) {
        line = cleanCsvCell(line);

        if (line.empty()) {
            continue;
        }

        std::vector<float> row;
        std::stringstream ss(line);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stof(cleanCsvCell(cell)));
        }

        rows.push_back(row);
    }

    if (rows.empty()) {
        throw std::runtime_error("empty matrix file " + path);
    }

    const size_t cols = rows[0].size();

    for (const auto& row : rows) {
        if (row.size() != cols) {
            throw std::runtime_error(
                "inconsistent number of columns in " + path
            );
        }
    }

    Eigen::MatrixXf M(rows.size(), cols);

    for (size_t i = 0; i < rows.size(); ++i) {
        for (size_t j = 0; j < cols; ++j) {
            M(i, j) = rows[i][j];
        }
    }

    return M;
}

inline Eigen::VectorXf loadVectorCSV(const std::string& path)
{
    Eigen::MatrixXf M = loadMatrixCSV(path);

    if (M.cols() == 1) {
        return M.col(0);
    }

    if (M.rows() == 1) {
        return M.row(0).transpose();
    }

    throw std::runtime_error("expected a vector in " + path);
}

struct CsvTable
{
    std::vector<std::string> header;
    std::vector<std::vector<float>> rows;
};

inline CsvTable loadCsvTable(const std::string& path)
{
    std::ifstream f(path);

    if (!f.is_open()) {
        throw std::runtime_error("cannot open " + path);
    }

    CsvTable t;
    std::string line;
    bool first = true;

    while (std::getline(f, line)) {

        // Remove Windows-style carriage returns and whitespace.
        line = cleanCsvCell(line);

        if (line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        std::string cell;

        if (first) {
            // Read column names.
            while (std::getline(ss, cell, ',')) {
                t.header.push_back(cleanCsvCell(cell));
            }

            first = false;
        }
        else {
            // Read numerical data.
            std::vector<float> row;

            while (std::getline(ss, cell, ',')) {
                row.push_back(std::stof(cleanCsvCell(cell)));
            }

            if (row.size() != t.header.size()) {
                throw std::runtime_error(
                    "column count mismatch in " + path
                );
            }

            t.rows.push_back(row);
        }
    }

    if (t.header.empty()) {
        throw std::runtime_error("missing CSV header in " + path);
    }

    if (t.rows.empty()) {
        throw std::runtime_error("missing CSV data in " + path);
    }

    return t;
}

inline int colIndex(const CsvTable& t, const std::string& name)
{
    const std::string cleanedName = cleanCsvCell(name);

    for (size_t i = 0; i < t.header.size(); ++i) {
        if (cleanCsvCell(t.header[i]) == cleanedName) {
            return static_cast<int>(i);
        }
    }

    std::string available;

    for (const auto& header : t.header) {
        if (!available.empty()) {
            available += ", ";
        }

        available += "[" + header + "]";
    }

    throw std::runtime_error(
        "column not found: " + name +
        ". Available columns: " + available
    );
}
