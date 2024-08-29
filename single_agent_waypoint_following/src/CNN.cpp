#include "single_agent_waypoint_following/CNN.hpp"
// tmp
#include <rclcpp/rclcpp.hpp>

// is a phenotype or representation of a genome
// takes input, applies activation, produces output

CNNNeuron::CNNNeuron(size_t num_inputs): num_inputs_(num_inputs+1)
{
    // we need an additional weight for the bias hence the +1
    for (size_t i=0; i<num_inputs+1; ++i)
    {
        // set up the weights with an initial random value
        weights_.push_back(utils::generateRangeRandom(-1.0, 1.0));
    }
}

CNNNeuron::CNNNeuron(size_t num_inputs, const std::vector<double>& weights): num_inputs_(num_inputs+1)
{
    // we need an additional weight for the bias hence the +1
    weights_ = weights;
}

CNNLayer::CNNLayer(size_t neurons, size_t inputs, const std::vector<double>& weights, size_t min_range, size_t max_range):num_neurons_(neurons)
{
    size_t weights_per_neuron = (max_range-min_range) / neurons;
    std::vector<double> holder(weights_per_neuron);

    // tmp
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "max_range-min_range: %ld", max_range-min_range);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "weights_per_neuron: %ld", weights_per_neuron);
    
    size_t i = min_range;
    size_t j = min_range + weights_per_neuron;
    for (size_t k=0; k<neurons; ++k) {
        std::copy(weights.begin() + i, weights.begin() + j, holder.begin());

        // tmp
        for (size_t i=0; i<holder.size(); ++i) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "neuron[%ld]->weights[%ld]: %f", k, i, holder[i]);
        }

        neurons_.push_back(CNNNeuron(inputs, holder));
        i = j;
        j += weights_per_neuron;
    }
}

// bias weight is not considered in parameters and is applied manually
CNN::CNN(size_t inputs, size_t outputs, size_t hidden, size_t in_hidden, const std::vector<double>& weights)
{
    num_inputs_ = inputs;
    num_outputs_ = outputs;
    num_hidden_ = hidden;
    neurons_per_hidden_ = in_hidden;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CNN::CNN():weights.size(): %ld", weights.size());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CNN::CNN():num_hidden: %ld", num_hidden_);

    if (num_hidden_ != 0)
        assert(weights.size() == (inputs+1)*in_hidden + (hidden-1)*in_hidden*(in_hidden+1) + outputs*(in_hidden+1));
    else
        assert(weights.size() == (inputs+1)*outputs);
    
    create_net(weights);
}

void CNN::create_net(const std::vector<double>& weights)
{
    // tmp
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "weights:");
    // for (size_t i=0; i<weights.size(); ++i) {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "weights[%ld]: %f", i, weights[i]);
    // }

    if (num_hidden_ > 0)
    {
        //create first hidden layer
        layers_.push_back(CNNLayer(neurons_per_hidden_, num_inputs_, weights, 0, (num_inputs_+1)*neurons_per_hidden_));

        size_t min_range = (num_inputs_+1)*neurons_per_hidden_;
        size_t max_range = min_range + (neurons_per_hidden_+1)*neurons_per_hidden_;

        for (size_t i=0; i<num_hidden_-1; ++i) {
            layers_.push_back(CNNLayer(neurons_per_hidden_, neurons_per_hidden_, weights, min_range, max_range));
            min_range = max_range;
            if (i != num_hidden_-2)
                max_range = min_range + (neurons_per_hidden_+1)*neurons_per_hidden_;
            else
                max_range = min_range + (neurons_per_hidden_+1)*num_outputs_;
        }
        
        //create output layer
        layers_.push_back(CNNLayer(num_outputs_, neurons_per_hidden_, weights, min_range, max_range));
    }
    else
    {
        //create output layer
        layers_.push_back(CNNLayer(num_outputs_, num_inputs_, weights, 0, (num_inputs_+1)*num_outputs_));
    }
}

std::vector<double> CNN::update(std::vector<double> &inputs)
{
    std::vector<double> outputs(num_outputs_);

    //first check that we have the correct amount of inputs
    if (inputs.size() != num_inputs_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "CNN::update: inputs.size() != num_inputs_");
        //just return an empty vector if incorrect.
        return outputs;
    }

    //For each layer...
    for (size_t i=0; i<num_hidden_ + 1; ++i)
    {
        if ( i > 0 )
            inputs = outputs;
        
        outputs.clear();

        // WARN: REMOVED THIS. WHY DOES HE USE IT?
        // cWeight = 0;

        for (size_t j=0; j<layers_[i].num_neurons_; ++j)
        {
            double score = 0;
            size_t n = layers_[i].neurons_[j].num_inputs_;

            for (size_t k=0; k<n-1; ++k)
            {
                score += layers_[i].neurons_[j].weights_[k] * inputs[k];
            }


            //add in the bias
            score += layers_[i].neurons_[j].weights_[n-1] * CNN::Bias;

            double a = utils::sigmoid(score, CNN::ActivationResponse);
            // double a = std::tanh(score);
            outputs.push_back(a);
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "CNN::update: a: %f, i:%ld, j:%ld, n-1:%ld", a, i, j, n-1);
        }
    }

    return outputs;
}

std::vector<size_t> CNN::get_split_points() const
{
    std::vector<size_t> boundaries;
    size_t wcounter = 0;

    for (size_t i=0; i<num_hidden_ + 1; ++i)
    {
        for (size_t j=0; j<layers_[i].num_neurons_; ++j)
        {
            for (size_t k=0; k<layers_[i].neurons_[j].num_inputs_; ++k)
            {
                ++wcounter;
            }

            boundaries.push_back(wcounter-1);
        }
    }

    return boundaries;
}