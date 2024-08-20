#ifndef EVOLVER_HPP
#define EVOLVER_HPP

#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include <queue>

#include "genome.hpp"

// template <typename T>
// double generateRandom(T radius) {
//     std::random_device seed;

//     // distribution over closed interval
//     std::uniform_real_distribution d(0.0, (double)radius);
//     auto gen = std::bind(d, std::mt19937(seed()));
    
//     return gen();
// }

template <typename T, size_t Length>
class Evolver
{
    struct Candidates {
        size_t first;
        size_t second;
        double combined_fitness;

        public:
            // Candidates(size_t first, size_t second, double f) {
            //     this->first = first;
            //     this->second = second;
            //     this->combined_fitness = f;
            // }

            bool operator() (Candidates& c1, Candidates& c2)
            {
                return c1.combined_fitness < c2.combined_fitness;
            }
    };

    public:
        Evolver(size_t size, double limit, const std::vector<T>& seed) : limit_(limit), seed_(seed) {
            population.resize(size);
        }

        void init_population() {
            for (size_t i=0; i<population.size(); ++i) {
                population[i] = generate_random_genome();
            }
        }

        std::vector<Genome<T, Length>>& get_population()
        {
            return this->population;
        } 

        void mutate(size_t index) {
            population[index].mutate();
        }

        std::priority_queue<Candidates, std::vector<Candidates>, Candidates> generate_fittest_pairs(size_t n) {
            std::priority_queue<Candidates, std::vector<Candidates>, Candidates> pairs;

            // for (size_t i=0; i<population.size(); ++i) {
            //     for (size_t j=i+1; j<population.size(); ++j) {
            // Start from the end of the sorted vector
            for (size_t i = n - 1; i > 0; --i) {
                for (size_t j = i - 1; j < n; --j) {
                    double fitness_combined = population[i].fitness + population[j].fitness;
                    Candidates cand;
                    cand.first = i;
                    cand.second = j;
                    cand.combined_fitness = fitness_combined;
                    pairs.push(cand);
                    // pairs.push({i, j, fitness_combined});
                    if (pairs.size() == n - 1) // the last will be replicated
                        return pairs;
                }
            }

            return pairs;
        }

        double rank_candidates()
        {
            // ascending order due to Genome comparison operator definition
            std::sort(population.begin(), population.end(), Genome<T, Length>::compare);
            return population[population.size()-1].fitness;
        }

        void generate_new_population() {
            // // ascending order due to Genome comparison operator definition
            // std::sort(population.begin(), population.end(), Genome<T, Length>::compare);

            // priority queue is descending
            std::priority_queue<Candidates, std::vector<Candidates>, Candidates> pairs = generate_fittest_pairs(population.size());

            std::vector<Genome<T, Length>> new_population;

            // for (size_t i=0; i<population.size()-1; ++i) {
            //     new_population[i] = population[i].crossover(population[i+1]);
            // }
            while (!pairs.empty()) {
                const auto& pair = pairs.top();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pair: {1st:%ld, 2nd:%ld, f:%f}", pair.first, pair.second, pair.combined_fitness);

                new_population.push_back(population[pair.first].crossover(population[pair.second]));
                pairs.pop();
            }

            // replication of the fittest
            new_population.push_back(population[population.size()-1]);
            new_population[population.size()-1].fitness = 0.0;
            population = new_population;
        }

        std::vector<Genome<T, Length>> population;

    private:
        Genome<T, Length> generate_random_genome() {
            std::array<T, Length> genes;

            for (size_t i=0; i<Length; ++i) {
                genes[i] = seed_[i] + generateRandom(limit_);
            }
            return Genome<T, Length>(genes);
        }

        // private data
        std::vector<T> seed_;
        T limit_;
};

#endif
