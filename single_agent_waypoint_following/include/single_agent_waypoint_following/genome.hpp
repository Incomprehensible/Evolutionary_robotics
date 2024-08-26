#ifndef GENOME_HPP
#define GENOME_HPP

#include <memory>
#include <string>
#include <array>
#include <cstring>

#include "utils.hpp"

// using CRTP (Curiously Recurring Template Pattern)

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

        Genome() = default;
        Genome(const std::array<T, Length>& genes) {
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

        GDerived crossover(const Genome<T, Length, GDerived>& other) {
            return static_cast<const GDerived*>(this)->crossover_impl(other);
        }
        
        void mutate() {
            static_cast<GDerived*>(this)->mutate_impl();
        }

        std::array<Genome_t, Length> genes_;
        double fitness = 0.0;
};

#endif