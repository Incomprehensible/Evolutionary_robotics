#ifndef CNN_GENOME_HPP
#define CNN_GENOME_HPP

#include "genome.hpp"


template <typename T, size_t Weights>
struct CNNGenome: public Genome<T, Weights, CNNGenome<T, Weights>>
{
    static constexpr double MutationRate = 0.5;
    static constexpr double MaxPerturbation = 1.0;//0.1;
    static constexpr double GeneMutationRate = 0.07;

    public:
        CNNGenome(): Genome<T, Weights, CNNGenome>()
        {
            num_inputs_ = 0;
            num_outputs_ = 0;
            num_hidden_ = 0;
            neurons_per_hidden_ = 0;
        }

        CNNGenome(size_t inputs, size_t outputs, size_t hidden, size_t in_hidden): Genome<T, Weights, CNNGenome>() {
            num_inputs_ = inputs;
            num_outputs_ = outputs;
            num_hidden_ = hidden;
            neurons_per_hidden_ = in_hidden;

            if (num_hidden_ != 0)
                assert(Weights == (inputs+1)*in_hidden + (hidden-1)*in_hidden*(in_hidden+1) + outputs*(in_hidden+1));
            else
                assert(Weights == (inputs+1)*outputs);
        }

        CNNGenome(const std::array<T, Weights>& genes) : Genome<T, Weights, CNNGenome>(genes) {
            num_inputs_ = 0;
            num_outputs_ = 0;
            num_hidden_ = 0;
            neurons_per_hidden_ = 0;
        }

        // CNNGenome(const CNNGenome& other) : Genome<T, Weights, CNNGenome>(other.genes_)
        // {
        //     num_inputs_ = other.num_inputs_;
        //     num_outputs_ = other.num_outputs_;
        //     num_hidden_ = other.num_hidden_;
        //     neurons_per_hidden_ = other.neurons_per_hidden_;
        // }

        // mutate CNN weights
        void mutate_impl()
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mutation!");

            //traverse the weight vector and mutate each weight dependent
            //on the mutation rate
            for (size_t i=0; i<this->genes_.size(); ++i)
            {
                //do we perturb this weight?
                if (utils::generateChance(CNNGenome::GeneMutationRate))
                {
                    //add or subtract a small value to the weight
                    this->genes_[i].raw_value += (utils::generateRangeRandom(-17.0, 17.0) * CNNGenome::MaxPerturbation);
                    // this->genes_[i].raw_value += (utils::generateRangeRandom(-1.0, 1.0) * CNNGenome::MaxPerturbation);
                }
            }
        }

        void crossover_impl(const Genome<T, Weights, CNNGenome>& other, Genome<T, Weights, CNNGenome>& child1, Genome<T, Weights, CNNGenome>& child2)
        {
            std::vector<size_t> boundaries = get_split_points();

            //determine two crossover points
            size_t index1 = utils::generateRangeRandom((int)0, (int)boundaries.size()-2);
            size_t index2 = utils::generateRangeRandom((int)index1, (int)boundaries.size()-1);
            size_t cp1 = boundaries[index1];
            size_t cp2 = boundaries[index2];

            // create the offspring
            for (size_t i=0; i<Weights; ++i)
            {
                if ( (i < cp1) || (i >= cp2) )
                {
                    // if outside of crossover points
                    child1.genes_[i].raw_value = this->genes_[i].raw_value;
                    child2.genes_[i].raw_value = other.genes_[i].raw_value;
                }
                else
                {
                    //switch over
                    child1.genes_[i].raw_value = other.genes_[i].raw_value;
                    child2.genes_[i].raw_value = this->genes_[i].raw_value;
                }
            }
            child1.increment_id();
            child2.increment_id();

            return;
        }

        CNNGenome<T, Weights> crossover_impl(const Genome<T, Weights, CNNGenome>& other) const {
            CNNGenome<T, Weights> new_genome(num_inputs_, num_outputs_, num_hidden_, neurons_per_hidden_);
            std::vector<size_t> boundaries = get_split_points();

            //determine two crossover points
            size_t index1 = utils::generateRangeRandom((int)0, boundaries.size()-2);
            size_t index2 = utils::generateRangeRandom((int)index1, boundaries.size()-1);
            size_t cp1 = boundaries[index1];
            size_t cp2 = boundaries[index2];

            for (size_t i=0; i<Weights; ++i) {
                if ( (i < cp1) || (i >= cp2) ) {
                    new_genome.genes_[i].raw_value = this->genes_[i].raw_value;
                }
                else {
                    T g1 = other.genes_[i].raw_value;
                    T g2 = this->genes_[i].raw_value;
    
                    // Perform arithmetic crossover
                    // double alpha = generateRandom(1.0); // was successfull
                    T alpha = utils::generateRangeRandom(0.5, 1.0);
                    new_genome.genes_[i].raw_value = alpha * g1 + (1.0 - alpha) * g2;
                }
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new_genome.genes_[%zu].raw_value: %f", i, new_genome.genes_[i].raw_value);
            }
  
            return new_genome;
        }

        void set_request_impl(std::shared_ptr<agent_interface::srv::SetConfig::Request>& request)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "config:\nnum_inputs: %ld, num_outputs: %ld, num_hidden: %ld, per_hidden: %ld", num_inputs_, num_outputs_, num_hidden_, neurons_per_hidden_);

            request->cnnconf.num_inputs = num_inputs_;
            request->cnnconf.num_outputs = num_outputs_;
            request->cnnconf.num_hidden = num_hidden_;
            request->cnnconf.neurons_per_hidden = neurons_per_hidden_;

            if (!request->cnnconf.weights.empty())
                request->cnnconf.weights.clear();
            
            for (size_t i=0; i<Weights; ++i)
                request->cnnconf.weights.push_back(this->genes_[i].raw_value);
        }

        static CNNGenome<T, Weights> generate_random_genome_impl(std::shared_ptr<agent_interface::srv::SetConfig::Request>& config)
        {
            CNNGenome<T, Weights> g(config->cnnconf.num_inputs, config->cnnconf.num_outputs, config->cnnconf.num_hidden, config->cnnconf.neurons_per_hidden);
            std::array<T, Weights> genes;

            for (size_t i=0; i<Weights; ++i)
            {
                // set up the weights with an initial random value
                genes[i] = utils::generateRangeRandom(-17.0, 17.0);
                // genes[i] = utils::generateRangeRandom(-1.0, 1.0);
            }
            g.put_genes(genes);

            return g;
        }


    private:
        std::vector<size_t> get_split_points() const
        {
            std::vector<size_t> boundaries;
            size_t wcounter = 0;

            // first hidden layer of neurons takes num_inputs_ (= each neuron's number of weights)
            if (num_hidden_ >= 1) {
                for (size_t j=0; j<neurons_per_hidden_; ++j)
                {
                    for (size_t k=0; k<num_inputs_+1; ++k)
                        ++wcounter;

                    boundaries.push_back(wcounter-1);
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Boundary: %ld", wcounter-1);
                }

                for (size_t i=0; i<num_hidden_-1; ++i)
                {
                    for (size_t j=0; j<neurons_per_hidden_; ++j)
                    {
                        for (size_t k=0; k<neurons_per_hidden_+1; ++k)
                        {
                            ++wcounter;
                        }

                        boundaries.push_back(wcounter-1);
                        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Boundary: %ld", wcounter-1);
                    }
                }

                for (size_t j=0; j<num_outputs_; ++j)
                {
                    for (size_t k=0; k<neurons_per_hidden_+1; ++k)
                    {
                        ++wcounter;
                    }
                    boundaries.push_back(wcounter-1);
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Boundary: %ld", wcounter-1);
                }
            }
            else {
                for (size_t j=0; j<num_outputs_; ++j)
                {
                    for (size_t k=0; k<num_inputs_+1; ++k)
                    {
                        ++wcounter;
                    }
                    boundaries.push_back(wcounter-1);
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Boundary: %ld", wcounter-1);
                }
            }

            return boundaries;
        }

        size_t num_inputs_;
        size_t num_outputs_;
        size_t num_hidden_;
        size_t neurons_per_hidden_;
};

#endif
