// main.cpp
#include <fdeep/fdeep.hpp>
#include <iostream>
#include <chrono>


int main()
{
    const auto model = fdeep::load_model("../export_model/fdeep_model.json");

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    const auto result = model.predict(
        {fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(6 + 4 + 3 + 3)),
                       std::vector<double>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1})});
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;


    std::cout << fdeep::show_tensors(result) << std::endl;
}
