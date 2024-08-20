#ifndef GENOME_HPP
#define GENOME_HPP

#include <memory>
#include <string>
#include <array>
#include <random>
#include <cstring>

// tmp
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <iomanip>

template <typename T>
double generateRandom(T radius) {
    std::random_device seed;

    // distribution over closed interval
    std::uniform_real_distribution d(0.0, (double)radius);
    auto gen = std::bind(d, std::mt19937(seed()));
    
    return gen();
}

template <typename T, size_t Length>
class Genome
{
    union Genome_t {
        T       raw_value;
        uint8_t asBytes[sizeof(T)];
    };

    public:
        static bool compare (Genome& g1, Genome& g2)
        {
            return g1.fitness < g2.fitness;
        }

        Genome() = default;
        Genome(const std::array<T, Length>& genes) {
            for (size_t i=0; i<Length; ++i) {
                genes_[i].raw_value = genes[i];
            }
        }

        void mutate() {
            std::random_device seed; // obtain a random number from hardware
            // distribution over closed interval
            std::uniform_int_distribution d_g(0, Length-1);
            std::uniform_int_distribution d_i(0, sizeof(T)-1);
            std::uniform_int_distribution d_j(0, sizeof(uint8_t)-1);
    
            auto gen_gene = std::bind(d_g, std::mt19937(seed()));
            auto gen_i = std::bind(d_i, std::mt19937(seed()));
            auto gen_j = std::bind(d_j, std::mt19937(seed()));

            size_t g = gen_gene();
            size_t i = gen_i();
            uint8_t& gene = genes_[g].asBytes[i];
            gene ^= (0b1 << gen_j());
        }

        // Genome<T, Length> crossover(const Genome& other) {
        //     Genome<T, Length> new_genome;
        //     uint8_t tmp[sizeof(T)] = {0};

        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crossover!");
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "other.genes_[0].raw_value: %f", other.genes_[0].raw_value);
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "other.genes_[1].raw_value: %f", other.genes_[1].raw_value);
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "this.genes_[0].raw_value: %f", genes_[0].raw_value);
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "this.genes_[1].raw_value: %f", genes_[1].raw_value);

        //     for (size_t i=0; i<Length; ++i) {
        //         for (size_t j=0; j<sizeof(T); ++j) {
        //             uint8_t g1 = other.genes_[i].asBytes[j];
        //             uint8_t g2 = this->genes_[i].asBytes[j];
        //             // tmp[j] = (g1 & 0xF0) | (g2 >> 4);
        //             tmp[j] = (g1 & 0xF0) | (g2 & 0x0F);
        //         }

        //         memcpy(new_genome.genes_[i].asBytes, tmp, sizeof(T));

        //         // tmp
        //         std::ostringstream oss;
        //         for (size_t j = 0; j < sizeof(T); ++j) {
        //             oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(new_genome.genes_[i].asBytes[j]);
        //         }
        //         std::string str = oss.str();
        //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new_genome.genes_[%zu].asBytes: %s", i, str.c_str());
        //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new_genome.genes_[%zu].raw_value: %f", i, new_genome.genes_[i].raw_value);
        //         //
        //     }
        //     return new_genome;
        // }

        Genome<T, Length> crossover(const Genome& other) {
            Genome<T, Length> new_genome;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crossover!");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "other.genes_[0].raw_value: %f", other.genes_[0].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "other.genes_[1].raw_value: %f", other.genes_[1].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "this.genes_[0].raw_value: %f", genes_[0].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "this.genes_[1].raw_value: %f", genes_[1].raw_value);

            for (size_t i=0; i<Length; ++i) {
                double g1 = other.genes_[i].raw_value;
                double g2 = this->genes_[i].raw_value;

                // Perform arithmetic crossover
                // double alpha = 0.5;
                double alpha = generateRandom(1.0);
                new_genome.genes_[i].raw_value = alpha * g1 + (1.0 - alpha) * g2;
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new_genome.genes_[%zu].raw_value: %f", i, new_genome.genes_[i].raw_value);
            }
            return new_genome;
        }


    // private:
        std::array<Genome_t, Length> genes_;
        double fitness = 0.0;
};

#endif