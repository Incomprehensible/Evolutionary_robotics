#ifndef CNN_HPP
#define CNN_HPP

#include <vector>
#include <string>

#include "utils.hpp"

// ■ The minesweeper’s position (x, y)
// ■ The position of the closest mine (x, y)
// ■ A vector representing the minesweeper’s heading (x, y)
// This makes a total of six inputs.

struct CNNNeuron
{
    public:
        // the number of inputs into the neuron
        size_t num_inputs_;
        CNNNeuron(size_t);
        CNNNeuron(size_t, const std::vector<double>&);

    private:
        // the weights for each input
        std::vector<double> weights_;
    
    friend class CNN;
};

struct CNNLayer
{
    public:
        // the number of neurons in this layer
        size_t num_neurons_;
        CNNLayer(size_t, size_t, const std::vector<double>&, size_t, size_t);

    private:
        // the layer of neurons
        std::vector<CNNNeuron> neurons_;
    
    friend class CNN;
};

class CNN
{
    static constexpr double Bias = -1.0;
    static constexpr double ActivationResponse = 1.0;

    public:
        CNN(size_t, size_t, size_t, size_t, const std::vector<double>&);
        std::vector<double> get_weights() const;
        size_t get_weights_num() const;
        void put_weights(std::vector<double>&);
        std::vector<double> update(std::vector<double>&);
        std::vector<size_t> get_split_points() const;

        
    private:
        void create_net(const std::vector<double>&);

        size_t num_inputs_;
        size_t num_outputs_;
        size_t num_hidden_;
        size_t neurons_per_hidden_;

        // storage for each layer of neurons including the output layer
        std::vector<CNNLayer> layers_;
};


#endif