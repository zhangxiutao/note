#include <iostream>
using namespace std;

typedef struct gaussian_parameter
{
	cv::Point mu;
	cv::Point sigma;
}gaussian_parameter;

gaussian_parameter update(gaussian_parameter prior, gaussian_parameter measurement)
{
	gaussian_parameter updated_parameter;
	updated_parameter.mu = cv::Point((prior.sigma.x * measurement.mu.x + measurement.sigma.x * prior.mu.x) / (prior.sigma.x + measurement.sigma.x),
	(prior.sigma.y * measurement.mu.y + measurement.sigma.y * prior.mu.y) / (prior.sigma.y + measurement.sigma.y));
	updated_parameter.sigma = cv::Point(1 / (1 / measurement.sigma.x + 1 / prior.sigma.x), 1 / (1 / measurement.sigma.y + 1 / prior.sigma.y));
	return updated_parameter;
}

gaussian_parameter predict(gaussian_parameter estimation, gaussian_parameter velocity)
{
	gaussian_parameter predicted_parameter;
	predicted_parameter.mu = cv::Point(estimation.mu.x + velocity.mu.x, estimation.mu.y + velocity.mu.y);
	predicted_parameter.sigma = cv::Point(estimation.sigma.x + velocity.sigma.x, estimation.sigma.y + velocity.sigma.y);
	return predicted_parameter;
}

void predictTrajectory(cv::Point* velocity_vectors, cv::Point* predicted_trajectory, const unsigned int velocity_vectors_buffer_size)
{
	gaussian_parameter positions[velocity_vectors_buffer_size + 1];
	positions[0] = gaussian_parameter{(cv::Point(0, 0), cv::Point(10000, 10000)};
	
	for (int i = 0; i < velocity_vectors_buffer_size; i++)
	{
		positions[i + 1] = predict(position[i], velocity_vectors[i]);
	}
	
}

void main()
{
 
}