#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>

inline Eigen::MatrixXf loadMatrixCSV(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("cannot open " + path);
    std::vector<std::vector<float>> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::vector<float> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
        rows.push_back(row);
    }
    if (rows.empty()) throw std::runtime_error("empty matrix file " + path);
    Eigen::MatrixXf M(rows.size(), rows[0].size());
    for (size_t i = 0; i < rows.size(); ++i)
        for (size_t j = 0; j < rows[i].size(); ++j)
            M(i,j) = rows[i][j];
    return M;
}

inline Eigen::VectorXf loadVectorCSV(const std::string& path) {
    Eigen::MatrixXf M = loadMatrixCSV(path);
    if (M.cols() == 1) return M.col(0);
    if (M.rows() == 1) return M.row(0).transpose();
    throw std::runtime_error("expected a vector in " + path);
}

struct CsvTable {
    std::vector<std::string> header;
    std::vector<std::vector<float>> rows;
};

inline CsvTable loadCsvTable(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("cannot open " + path);
    CsvTable t;
    std::string line;
    bool first = true;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        if (first) {
            while (std::getline(ss, cell, ',')) t.header.push_back(cell);
            first = false;
        } else {
            std::vector<float> row;
            while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
            t.rows.push_back(row);
        }
    }
    return t;
}

inline int colIndex(const CsvTable& t, const std::string& name) {
    for (size_t i = 0; i < t.header.size(); ++i)
        if (t.header[i] == name) return (int)i;
    throw std::runtime_error("column not found: " + name);
}