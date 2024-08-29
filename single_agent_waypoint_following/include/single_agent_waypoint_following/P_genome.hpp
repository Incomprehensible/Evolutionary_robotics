#ifndef P_GENOME
#define P_GENOME

#include "genome.hpp"

// tmp
#include <rclcpp/rclcpp.hpp>

template <typename T, size_t Length>
class PGenome: public Genome<T, Length, PGenome<T, Length>>
{
    public:

    static constexpr double MutationRate = 0.15;

    PGenome(): Genome<T, Length, PGenome>() {}
    PGenome(const std::array<T, Length>& genes) : Genome<T, Length, PGenome>(genes) {}

    void mutate_impl() {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mutation!");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BEFORE: p.genes_[0].raw_value: %f", this->genes_[0].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BEFORE: p.genes_[1].raw_value: %f", this->genes_[1].raw_value);

            std::random_device seed; // obtain a random number from hardware
            // distribution over closed interval
            std::uniform_int_distribution d_g(0, 1);
            std::uniform_int_distribution d_i(4, (int)sizeof(double)-2);
            std::uniform_int_distribution d_j(0, 7);
    
            auto gen_gene = std::bind(d_g, std::mt19937(seed()));
            auto gen_i = std::bind(d_i, std::mt19937(seed()));
            auto gen_j = std::bind(d_j, std::mt19937(seed()));

            size_t g = gen_gene();
            size_t i = gen_i();
            
            this->genes_[g].asBytes[i] ^= (0b1 << gen_j());

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "g: %ld, i: %ld", g, i);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AFTER: p.genes_[0].raw_value: %f", this->genes_[0].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AFTER: p.genes_[1].raw_value: %f", this->genes_[1].raw_value);
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

        PGenome<T, Length> crossover_impl(const Genome<T, Length, PGenome>& other) const {
            PGenome<T, Length> new_genome;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crossover!");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "other.genes_[0].raw_value: %f", other.genes_[0].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "other.genes_[1].raw_value: %f", other.genes_[1].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "this.genes_[0].raw_value: %f", this->genes_[0].raw_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "this.genes_[1].raw_value: %f", this->genes_[1].raw_value);

            for (size_t i=0; i<Length; ++i) {
                T g1 = other.genes_[i].raw_value;
                T g2 = this->genes_[i].raw_value;

                // Perform arithmetic crossover
                // double alpha = generateRandom(1.0); // was successfull
                T alpha = utils::generateRangeRandom(0.5, 1.0);
                new_genome.genes_[i].raw_value = alpha * g1 + (1.0 - alpha) * g2;
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new_genome.genes_[%zu].raw_value: %f", i, new_genome.genes_[i].raw_value);
            }
            return new_genome;
        }

        void crossover_impl(const Genome<T, Length, PGenome>& other, Genome<T, Length, PGenome>& child1, Genome<T, Length, PGenome>& child2)
        {
            // create the offspring
            for (size_t i=0; i<Length; ++i)
            {
                if (i % 2 == 0) {
                    child1.genes_[i].raw_value = this->genes_[i].raw_value;
                    child2.genes_[i].raw_value = other.genes_[i].raw_value;
                }
                else {
                    child1.genes_[i].raw_value = other.genes_[i].raw_value;
                    child2.genes_[i].raw_value = this->genes_[i].raw_value;
                }
            }

            return;
        }

        void set_request_impl(std::shared_ptr<agent_interface::srv::SetConfig::Request>& request)
        {
            request->pconf.k_l = this->genes_[0].raw_value;
            request->pconf.k_ha = this->genes_[1].raw_value;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Candidate: K_l: {%f}, K_ha: {%f}, Fitness: {%f}", request->pconf.k_l, request->pconf.k_ha, this->fitness);
        }

        static PGenome<T, Length> generate_random_genome_impl(std::shared_ptr<agent_interface::srv::SetConfig::Request>& config) {
            std::array<T, Length> genes;
            genes[0] = config->pconf.k_l;
            genes[1] = config->pconf.k_ha;

            return PGenome<T, Length>(genes);
        }
};

#endif