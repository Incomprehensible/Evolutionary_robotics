#ifndef EVOLVER_HPP
#define EVOLVER_HPP

#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include <queue>

#include "CNN_genome.hpp"
#include "P_genome.hpp"

template <class G>
class Evolver
{
    using GeneType = typename G::GeneType;
    static constexpr size_t GeneLength = G::GeneLength;

    struct Candidates {
        size_t first;
        size_t second;
        double combined_fitness;

        public:
            bool operator() (Candidates& c1, Candidates& c2)
            {
                return c1.combined_fitness < c2.combined_fitness;
            }
    };

    public:
        Evolver(size_t size, GeneType limit, const std::vector<GeneType>& seed) : limit_(limit), seed_(seed) {
            population.resize(size);
        }

        void init_population() {
            for (size_t i=0; i<population.size(); ++i) {
                population[i] = G::generate_random_genome(seed_, limit_);
            }
        }

        void init_population_from_config(std::shared_ptr<agent_interface::srv::SetConfig::Request>& config) {
            for (size_t i=0; i<population.size(); ++i) {
                population[i] = G::generate_random_genome(config);
            }
        }

        std::vector<G>& get_population()
        {
            return this->population;
        } 

        void mutate(size_t index) {
            population[index].mutate();
        }

        std::priority_queue<Candidates, std::vector<Candidates>, Candidates> generate_fittest_pairs(size_t n) {
            std::priority_queue<Candidates, std::vector<Candidates>, Candidates> pairs;

            // Start from the end of the sorted vector
            for (size_t i = n - 1; i > 0; --i) {
                for (size_t j = i - 1; j < n; --j) {
                    double fitness_combined = population[i].fitness + population[j].fitness;
                    Candidates cand;
                    cand.first = i;
                    cand.second = j;
                    cand.combined_fitness = fitness_combined;
                    pairs.push(cand);
                    // if (pairs.size() == n - 1) // the last will be replicated
                    //     return pairs;
                }
            }

            return pairs;
        }

        double rank_candidates()
        {
            // ascending order due to Genome comparison operator definition
            std::sort(population.begin(), population.end(), G::compare);
            return population[population.size()-1].fitness;
        }

        void generate_new_population() {
            // priority queue is descending
            std::priority_queue<Candidates, std::vector<Candidates>, Candidates> pairs = generate_fittest_pairs(population.size());

            std::vector<G> new_population;

            while (new_population.size() != (population.size()-1) && !pairs.empty()) {
                const auto& pair = pairs.top();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pair: {1st:%ld, 2nd:%ld, f:%f}", pair.first, pair.second, pair.combined_fitness);

                // tmp
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parents:");
                population[pair.first].print_genes();
                population[pair.second].print_genes();
                //
                G p1 = population[pair.first];
                G p2 = population[pair.second];
                population[pair.first].crossover(population[pair.second], p1, p2);
                // if (utils::generateChance(G::MutationRate))
                //     p1.mutate();
                // if (utils::generateChance(G::MutationRate))
                //     p2.mutate();

                // tmp
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Children:");
                p1.print_genes();
                p2.print_genes();
                //

                new_population.push_back(p1);
                new_population.push_back(p2);
                pairs.pop();

                // alternative mutation
                // G p = population[pair.first].crossover(population[pair.second]);
                // if (utils::generateChance(G::MutationRate))
                //     p.mutate();
                // new_population.push_back(p);
                // pairs.pop();
            }
            // TODO: fix even numbers
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Population size: %ld", population.size());
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New population size: %ld", new_population.size());

            // replication of the fittest
            new_population.push_back(population[population.size()-1]);
            new_population[population.size()-1].fitness = 0.0;
            population = new_population;
        }

        std::vector<G> population;

    private:
        // private data
        GeneType limit_;
        std::vector<GeneType> seed_;
};

#endif
