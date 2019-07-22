#ifndef _FIR_FILTER_HPP_
#define _FIR_FILTER_HPP_

#include <iostream>
#include <string>
#include <deque>
#include <vector>

#include "filter_base.hpp"

class fir_filter : public filter_base {
public:
		fir_filter() = default;
    explicit fir_filter(const std::vector<double>& filter_coefficients);
    fir_filter(double* filter_coefficients, uint8_t filter_length);
    explicit fir_filter(const std::string& csv_filename);

    @Override
    void add_data(double new_data);
    @Override
    void clear_data();
    @Override
    double get_result();
private:
    std::vector<double> filter_coefficients;
    std::deque<double> data;
};

#endif
