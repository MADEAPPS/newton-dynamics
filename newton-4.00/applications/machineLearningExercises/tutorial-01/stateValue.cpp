// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <vector>
#include <random> 

int startingPosition = 1;
int cliffPosition = 0;
int endPosition = 5;

int cliffReward = 0;
int goalStateReward = 5;


static std::mt19937 generator;

float Rand()
{
	unsigned minValue = std::mt19937::min();
	unsigned maxValue = std::mt19937::max();
	unsigned spand(maxValue - minValue);
	float r = float(generator()) / (float)spand;
	return r;
}

bool IsTerminating(int currentPosition)
{
	if (currentPosition <= cliffPosition)
		return true;
	if (currentPosition >= endPosition)
		return true;
	return false;
}

int Strategy()
{
	float r = Rand();
	if (r > 0.5f)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

int Reward(int currentPosition)
{
	if (currentPosition <= cliffPosition)
	{
		return cliffReward;
	}
	else if (currentPosition >= endPosition)
	{
		return goalStateReward;
	}
	return 0;
}


int main()
{
	int n_iter = 10000;

	generator.seed(42);
	std::vector<int> hits;
	std::vector<int> valueSum;
	for (int i = 0; i < endPosition + 1; ++i)
	{
		hits.push_back(0);
		valueSum.push_back(0);
	}
	
	for (int i = 0; i < n_iter; ++i)
	{
		std::vector<int> positionHistory;
		int currentPosition = startingPosition;
		while (1)
		{
			positionHistory.push_back(currentPosition);
			if (IsTerminating(currentPosition))
			{
				break;
			}

			int value = Strategy();
			currentPosition += value;
		}

		int currentReward = Reward(currentPosition);

		for (int j = 0; j < int (positionHistory.size()); j++)
		{
			int pos = positionHistory[j];
			hits[pos] += 1;
			valueSum[pos] += currentReward;
		}

		printf("%d average reward: [", i);
		for (int j = 0; j < int (valueSum.size()); j++)
		{
			printf("%f ", float	(valueSum[j])/hits[j]);
		}
		printf("]\n");
	}

    return 0;
}

