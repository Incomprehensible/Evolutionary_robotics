#ifndef UTILS_HPP
#define UTILS_HPP

#include <random>

namespace utils {

template <typename T>
double generateRangeRandom(T min, T max) {
    std::random_device seed;

    // distribution over closed interval
    std::uniform_real_distribution d((double)min, (double)max);
    // auto gen = std::bind(d, std::mt19937(seed()));
    auto gen = std::bind(d, std::mt19937_64(seed()));
    
    return gen();
}

template <typename T>
double generateRandom(T radius) {
    return generateRangeRandom(0.0, radius);
}

double generateEdgeBiasedRandom(double radius) {
    // Generate a uniform random number between 0 and 1
    std::random_device seed; // obtain a random number from hardware

    // distribution over closed interval
    std::uniform_real_distribution d(0.0, 1.0);
    auto gen = std::bind(d, std::mt19937(seed()));
    double u = gen();
    
    // Map the uniform random number to the U-shaped distribution
    // This transformation squares the random variable and adjusts its sign.
    double biased_value = std::copysign(std::sqrt(2 * radius * std::abs(u - 0.5)), u - 0.5);
    
    // Scale the biased value to fit the radius [-radius, radius]
    double random_number = biased_value * radius;

    return random_number;
}

geometry_msgs::msg::Point generate_equidistant_setpoint(double radius)
{
    // randomly generate dx, dy, dyaw
    double theta = generateRandom(2*M_PI);

    geometry_msgs::msg::Point goal;
    goal.x = radius * std::cos(theta);
    goal.y = radius * std::sin(theta);
    goal.z = 0;
    return goal;
}

geometry_msgs::msg::Point generate_setpoint(double radius)
{
    // randomly generate dx, dy, dyaw
    double dx = generateEdgeBiasedRandom(radius);
    double dy = generateEdgeBiasedRandom(radius);

    geometry_msgs::msg::Point goal;
    goal.x = dx;
    goal.y = dy;
    goal.z = 0;
    return goal;
}

bool generateChance()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    // give "true" 1/10 of the time
    std::bernoulli_distribution d(0.15);
    return d(gen);
}
}

#endif