#ifndef GENOME_HPP
#define GENOME_HPP

#include <memory>
#include <string>
#include <array>
#include <cstring>

#include "agent_interface/srv/set_config.hpp"

#include "utils.hpp"

// tmp
#include <rclcpp/rclcpp.hpp>

// using CRTP (Curiously Recurring Template Pattern)

static size_t nextId = 0;

template <typename T, size_t Length, typename GDerived>
class Genome
{
    public:
    using GeneType = T;
    static constexpr size_t GeneLength = Length;

    union Genome_t {
        T       raw_value;
        uint8_t asBytes[sizeof(T)];
    };

        static bool compare (Genome<T, Length, GDerived>& g1, Genome<T, Length, GDerived>& g2)
        {
            return g1.fitness < g2.fitness;
        }

        Genome() { id_ = nextId++; }

        Genome(std::array<Genome_t, Length> genes) {
            genes_ = genes;
            id_ = nextId++;
        }

        Genome(const std::array<T, Length>& genes) {
            put_genes(genes);
            id_ = nextId++;
        }

        // tmp
        void increment_id() {
            id_ = nextId++;
        }

        void put_genes(const std::array<T, Length>& genes) {
            for (size_t i=0; i<Length; ++i) {
                genes_[i].raw_value = genes[i];
            }
        }

        static GDerived generate_random_genome(const std::vector<T>& seed, T limit) {
            std::array<T, Length> genes;

            for (size_t i=0; i<Length; ++i) {
                genes[i] = seed[i] + utils::generateRandom(limit);
            }
            return GDerived(genes);
        }

        static GDerived generate_random_genome(std::shared_ptr<agent_interface::srv::SetConfig::Request>& config) {
            return GDerived::generate_random_genome_impl(config);
        }

        GDerived crossover(const Genome<T, Length, GDerived>& other) {
            return static_cast<const GDerived*>(this)->crossover_impl(other);
        }

        void crossover(const Genome<T, Length, GDerived>& other, 
            Genome<T, Length, GDerived>& child1, Genome<T, Length, GDerived>& child2) {
                return static_cast<GDerived*>(this)->crossover_impl(other, child1, child2);
            // return static_cast<const GDerived*>(this)->crossover_impl(other, child1, child2);
        }
        
        void mutate() {
            static_cast<GDerived*>(this)->mutate_impl();
        }

        void set_request(std::shared_ptr<agent_interface::srv::SetConfig::Request>& request) {
            static_cast<GDerived*>(this)->set_request_impl(request);
        }

        void print_genes() {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Genome #%ld:", id_);

            for (size_t i=0; i<Length; ++i) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "genome.genes_[%zu].raw_value: %f", i, this->genes_[i].raw_value);
            }
        }

        size_t id_;
        std::array<Genome_t, Length> genes_;
        double fitness = 0.0;
};

#endif